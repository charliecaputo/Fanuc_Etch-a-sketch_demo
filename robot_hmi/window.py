#!/usr/bin/env python3

# ─────────────────────────────────────────────
# window.py
# Main HMI window — layout, button logic,
# process management, E-STOP state machine
# ─────────────────────────────────────────────

import subprocess
import os
import signal
import time
from io import StringIO

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QStackedWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QSizePolicy,
    QPlainTextEdit,
)
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QFont
from PyQt6.QtCore import QThread, pyqtSignal

from ros_bridge import ROSBridge
from widgets import ModeButton
from jog_page import JogPage
from joint_page import JointPage

# Page indices for QStackedWidget
PAGE_MAIN = 0
PAGE_JOG  = 1
PAGE_JOINT = 2

HOME_LABEL = "Home Robot"
HOME_SUB = "Return to home pose"

ENCODER_LABEL = "Encoder Teleop"
ENCODER_SUB = "XY workspace control"

ENABLE_LABEL = "Enable Robot"
ENABLE_SUB = "Start MoveIt stack"


class ProcessLogReader(QThread):
    line_received = pyqtSignal(str)

    def __init__(self, process):
        super().__init__()
        self.process = process
        self._running = True

    def run(self):
        if not self.process or not self.process.stdout:
            return

        while self._running:
            try:
                line = self.process.stdout.readline()
            except Exception:
                break

            if not line:
                self.msleep(10)
                continue

            self.line_received.emit(line.rstrip())

    def stop(self):
        self._running = False


class HMIWindow(QMainWindow):

    def __init__(self, ros: ROSBridge):
        super().__init__()

        self.ros = ros
        self.current_mode = 'IDLE'
        self.mode_buttons = {}
        self.home_btn = None
        self.encoder_btn = None
        self.enable_robot_btn = None

        # ── Process handles ──
        self.robot_process = None
        self.manual_init_process = None
        self.manual_init_running = False
        self.encoder_process = None
        self.encoder_processes = []
        self.encoder_running = False
        self.log_buffer = StringIO()
        self.last_flush = 0
        self.jog_listener_process = None

        # ── E-STOP state ──
        self.estop_latched = False
        self.estop_reset_ready = False
        self.estop_hold_time = 0
        self.estop_hold_required = 3000  # ms

        self.estop_hold_timer = QTimer()
        self.estop_hold_timer.setInterval(50)
        self.estop_hold_timer.timeout.connect(self._update_estop_hold)

        self.setWindowTitle("CRX-10iA Control")
        self.setFixedSize(1024, 600)
        self.show()
        #self.showFullScreen()
        #self.resize(960, 540)
        #self.show()
        

        self._build_ui()

        ros.mode_changed.connect(self._on_mode_changed)
        
        self.log_thread = None

    # =========================================================
    # Logging
    # =========================================================

    def _log(self, msg: str):
        self.term_output.appendPlainText(msg)
        self.term_output.ensureCursorVisible()
        

    # =========================================================
    # UI Construction
    # =========================================================

    def _build_ui(self):
        # ── Root container ──
        root_widget = QWidget()
        root_widget.setStyleSheet("background-color: #111111;")
        self.setCentralWidget(root_widget)

        root_layout = QVBoxLayout(root_widget)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        # ── Stacked pages ──
        self.stack = QStackedWidget()
        root_layout.addWidget(self.stack)

        # Page 0 — main control screen
        self.stack.addWidget(self._build_main_page())

        # Page 1 — cartesian jog screen
        self.jog_page = JogPage(self.ros, back_callback=self._show_main)
        self.stack.addWidget(self.jog_page)

        #Page 2 - Joint jog screen
        self.joint_page = JointPage(self.ros, back_callback=self._show_main)
        self.stack.addWidget(self.joint_page)
        

        self.stack.setCurrentIndex(PAGE_MAIN)
        
        

    def _build_main_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(16)

        layout.addLayout(self._build_header())
        layout.addWidget(self._build_status_bar())
        layout.addLayout(self._build_mode_grid())
        layout.addWidget(self._build_estop_button())

        return page

    def _build_header(self):
        header_row = QHBoxLayout()

        header = QLabel("FANUC CRX-10iA")
        header.setFont(QFont("Arial", 10))
        header.setStyleSheet("color: #555555;")

        min_btn = QPushButton("—")
        min_btn.setFixedSize(40, 40)
        min_btn.setStyleSheet("""
            QPushButton {
                background-color: #1a1a1a;
                color: #aaaaaa;
                border: 1px solid #444444;
                border-radius: 8px;
                font-size: 18px;
            }
            QPushButton:pressed {
                background-color: #333333;
            }
        """)
        min_btn.clicked.connect(self.showMinimized)

        close_btn = QPushButton("✕")
        close_btn.setFixedSize(40, 40)
        close_btn.setStyleSheet("""
            QPushButton {
                background-color: #330000;
                color: #ff5555;
                border: 1px solid #aa0000;
                border-radius: 8px;
                font-size: 18px;
                font-weight: bold;
            }
            QPushButton:pressed {
                background-color: #550000;
            }
        """)
        close_btn.clicked.connect(self.close)

        header_row.addWidget(header)
        header_row.addStretch()
        header_row.addWidget(min_btn)
        header_row.addWidget(close_btn)

        return header_row

    def _build_status_bar(self):
        self.status_bar = QWidget()
        self.status_bar.setStyleSheet(
            "background: #1e2a1e; border: 1px solid #2a4a2a; border-radius: 10px;"
        )

        status_layout = QVBoxLayout(self.status_bar)
        status_layout.setContentsMargins(16, 10, 16, 10)

        self.term_output = QPlainTextEdit()
        self.term_output.setReadOnly(True)
        self.term_output.setFont(QFont("Consolas", 12))
        self.term_output.setStyleSheet("""
            QPlainTextEdit {
                background-color: #0b0f0b;
                color: #33ff66;
                border: 1px solid #2a4a2a;
                border-radius: 8px;
            }
        """)
        self.term_output.setFixedHeight(80)
        self.term_output.setMaximumBlockCount(10)

        
        status_layout.addWidget(self.term_output)
        

        return self.status_bar
        
    def _run_jog_listener(self):
        if self.jog_listener_process and self.jog_listener_process.poll() is None:
            return  # already running
        
        self._kill_manual_init()
        self._kill_encoder_teleop()
        self.manual_init_running = False
        self.encoder_running = False

        if self.home_btn:
            self._reset_mode_button(
                self.home_btn,
                HOME_LABEL,
                HOME_SUB
            )

        if self.encoder_btn:
            self._reset_mode_button(
                self.encoder_btn,
                ENCODER_LABEL,
                ENCODER_SUB
            )
        cmd = "ros2 run test_py jog_listener_node"

        print("Starting jog listener...")
        self.jog_listener_process = self._launch_ros_process(cmd)
        
    
    def _kill_jog_listener(self):
        if not self.jog_listener_process:
            return

        if self.jog_listener_process.poll() is None:
            print("Stopping jog listener...")
            try:
                cmd = ("pkill -9 -f jog_listener_node")

                subprocess.Popen(
                        ["bash", "-c", cmd],
                        preexec_fn=os.setsid
                    )
            except Exception:
                self.jog_listener_process.terminate()
        
        self._reset_mode_button(
                self.manual_btn,
                "Manual Control",
                "MANUAL_JOG"
            )
        self.jog_listener_process = None
    
    def _reset_mode_button(self, btn, label, sub):
        btn.label_widget.setText(label)
        btn.sub_widget.setText(sub)
        btn._set_inactive()

    def _build_mode_grid(self):
        grid = QGridLayout()
        grid.setSpacing(12)
        
        callbacks = {
            "ENABLE_ROBOT": self._enable_robot,
            "HOME": self._run_manual_init,
            "ENCODER_TELEOP": self._run_encoder_teleop,
            "ABORT": self._abort_motion_only,
        }

        modes = [
            ("Encoder Teleop",  "XY workspace control",   "ENCODER_TELEOP", 0, 0),
            ("Home Robot",      "Return to home pose",     "HOME",           0, 1),
            ("ABORT",           "Stop running programs",   "ABORT",          1, 0),
            ("Enable Robot",    "Start MoveIt stack",      "ENABLE_ROBOT",   1, 1),
        ]

        for label, sub, key, row, col in modes:
            btn = ModeButton(label, sub, key)

            if key in callbacks:
                btn.clicked.connect(callbacks[key])

                if key == "ENABLE_ROBOT":
                    self.enable_robot_btn = btn
                elif key == "HOME":
                    self.home_btn = btn
                elif key == "ENCODER_TELEOP":
                    self.encoder_btn = btn
            else:
                btn.clicked.connect(
                    lambda checked, k=key: self.ros.send_mode(k)
                )
                self.mode_buttons[key] = btn

            grid.addWidget(btn, row, col)

        # ── Manual Control button — spans full width on row 2 ──
        manual_btn = ModeButton("Manual Control", "Cartesian jog XYZ", "MANUAL_JOG")
        manual_btn.clicked.connect(self._show_jog)
        self.manual_btn = manual_btn
        grid.addWidget(manual_btn, 2, 0, 1, 2)  # span both columns

        return grid
        
    def _launch_ros_process(
        self,
        command: str,
        capture_output: bool = False
    ):
        kwargs = {
            "preexec_fn": os.setsid
        }

        if capture_output:
            kwargs.update({
                "stdout": subprocess.PIPE,
                "stderr": subprocess.STDOUT,
                "text": True,
                "bufsize": 1
            })

        return subprocess.Popen(
            ["bash", "-c", command],
            **kwargs
        )

    def _build_estop_button(self):
        self.estop_btn = QPushButton("E-Stop")
        self.estop_btn.setFont(QFont("Arial", 18, QFont.Weight.Medium))
        self.estop_btn.setMinimumHeight(90)
        self._setup_estop_button()
        return self.estop_btn

    # =========================================================
    # Page navigation
    # =========================================================

    def _show_jog(self):
        self._run_jog_listener()
        self.stack.setCurrentIndex(PAGE_JOG)

    def _show_main(self, mode=None):
        if mode != "joint" and mode != "cartesian":
            self._kill_jog_listener()

        if mode == "joint":
            self.stack.setCurrentIndex(PAGE_JOINT)
            return

        if mode == "cartesian":
            self.stack.setCurrentIndex(PAGE_JOG)
            return

        self.stack.setCurrentIndex(PAGE_MAIN)

    # =========================================================
    # Mode callbacks
    # =========================================================

    def _on_mode_changed(self, mode: str):
        self.current_mode = mode
        #self.mode_label.setText(mode)
        for key, btn in self.mode_buttons.items():
            btn.set_active(key == mode)

    # =========================================================
    # Process management
    # =========================================================

    def _enable_robot(self):
        if self.robot_process is not None:
            if self.robot_process.poll() is None:
                print("Robot already running")
                return

        print("Launching robot...")
        self.current_mode = 'ENABLING ROBOT'
        #self.mode_label.setText(f"MODE: {self.current_mode}")

        self.enable_robot_btn.label_widget.setText("Starting")
        self.enable_robot_btn.sub_widget.setText("Launching MoveIt")

        cmd = (
            "source /opt/ros/jazzy/setup.bash && "
            "source /home/fanuc/fanuc_ws/install/setup.bash && "
            "ros2 launch test_py fac_moveit_test.launch.py use_mock:=true"
        )

        try:
            self.robot_process = self._launch_ros_process(
                cmd,
                capture_output=True
            )
            self.log_thread = ProcessLogReader(self.robot_process)
            self.log_thread.line_received.connect(self._log)
            self.log_thread.start()
            
            print(f"Robot launch started PID={self.robot_process.pid}")
            self.enable_robot_btn.label_widget.setText("Robot Enabled")
            self.enable_robot_btn.sub_widget.setText("MoveIt Running")
            self.current_mode = 'IDLE'
            #self.mode_label.setText(f"MODE: {self.current_mode}")

        except Exception as e:
            print(f"Launch failed: {e}")
            self.enable_robot_btn.label_widget.setText("Launch Failed")
            self.enable_robot_btn.sub_widget.setText("Check Terminal")

    def _run_manual_init(self):
        # ── STOP if already running ──
        if self.manual_init_running:
            print("Stopping manual init...")
            self._kill_manual_init()
            self._reset_mode_button(
                self.home_btn,
                HOME_LABEL,
                HOME_SUB
            )
            self.manual_init_running = False
            return

        # ── START ──
        self._kill_encoder_teleop()
        self._kill_jog_listener()
        self.encoder_running = False

        if self.manual_btn:
            self._reset_mode_button(
                self.manual_btn,
                "Manual Control",
                "MANUAL_JOG"
            )

        if self.encoder_btn:
            self._reset_mode_button(
                self.encoder_btn,
                ENCODER_LABEL,
                ENCODER_SUB
            )
        print("Starting manual init...")
        self.current_mode = 'MANUAL INITIALIZATION'
        #self.mode_label.setText(f"MODE: {self.current_mode}")

        self.home_btn.label_widget.setText("RUNNING")
        self.home_btn.sub_widget.setText(
            "Robot Moving to Home Pose (press again when robot is stopped)"
        )
        self.home_btn._set_active()

        cmd = (
            "source /opt/ros/jazzy/setup.bash && "
            "source /home/fanuc/fanuc_ws/install/setup.bash && "
            "python3 /home/fanuc/fanuc_ws/src/test_py/test_py/manual_init.py"
        )

        self.manual_init_process = self._launch_ros_process(cmd)
        self.manual_init_running = True

    def _run_encoder_teleop(self):
        # ── STOP if running ──
        if self.encoder_running:
            print("Stopping encoder teleop...")
            self._kill_encoder_teleop()
            self._reset_mode_button(
                self.encoder_btn,
                ENCODER_LABEL,
                ENCODER_SUB
            )
            self.encoder_running = False
            return

        # ── START ──
        self._kill_manual_init()
        self._kill_jog_listener()
        self.manual_init_running = False

        if self.home_btn:
            self._reset_mode_button(
                self.home_btn,
                HOME_LABEL,
                HOME_SUB
            )

        if self.encoder_btn:
            self._reset_mode_button(
                self.encoder_btn,
                ENCODER_LABEL,
                ENCODER_SUB
            )
        
        
        print("Starting encoder teleop...")
        self.current_mode = 'ENCODER TELEOP'
        #self.mode_label.setText(f"MODE: {self.current_mode}")

        self.encoder_btn.label_widget.setText("RUNNING")
        self.encoder_btn.sub_widget.setText(
            "Encoder + XY nodes active (press again to stop)"
        )
        self.encoder_btn._set_active()

        cmd = (
            "source /opt/ros/jazzy/setup.bash && "
            "source /home/fanuc/fanuc_ws/install/setup.bash && "
            "ros2 run fanuc_crx_xy_demo encoder_test & "
            "ros2 run test_py servo_control "
        )

        proc = self._launch_ros_process(cmd)
        self.encoder_processes.append(proc)
        self.encoder_running = True

    # ── Kill helpers ──

    def _terminate_process(self, proc):
        if not proc:
            return

        if proc.poll() is not None:
            return

        try:
            os.killpg(
                os.getpgid(proc.pid),
                signal.SIGTERM
            )
        except Exception:
            proc.terminate()

    def _kill_robot_stack(self):
        if self.robot_process is not None:
            if self.robot_process.poll() is None:
                print("Killing robot stack...")
                try:
                    os.killpg(os.getpgid(self.robot_process.pid), signal.SIGTERM)
                    #subprocess.run("pkill -f slider_gui_node", shell=True)
                    cmd = (
                        "pkill -9 -f slider_gui_node & "
                        "pkill -9 -f servo_node & "
                        "pkill -9 -f move_group"
                    )

                    subprocess.Popen(
                        ["bash", "-c", cmd],
                        preexec_fn=os.setsid
                    )
                    
                    if self.log_thread:
                        self.log_thread.stop()
                        self.log_thread.wait()
                        self.log_thread = None
                
                except Exception as e:
                    print(f"SIGTERM failed: {e}")
                    self.robot_process.terminate()
            self.robot_process = None

    def _kill_manual_init(self):
        print("Killing manual init process...")
        self._terminate_process(self.manual_init_process)
        self.manual_init_process = None

    def _kill_encoder_teleop(self):
        print("Killing encoder teleop processes...")

        for p in self.encoder_processes:
            self._terminate_process(p)

        self.encoder_processes = []
        self.encoder_running = False

    def _abort_motion_only(self):
        print("ABORT: stopping motion scripts only")
        self._kill_manual_init()
        self._kill_encoder_teleop()
        self.manual_init_running = False
        self.encoder_running = False

        if self.home_btn:
            self._reset_mode_button(
                self.home_btn,
                HOME_LABEL,
                HOME_SUB
            )

        if self.encoder_btn:
            self._reset_mode_button(
                self.encoder_btn,
                ENCODER_LABEL,
                ENCODER_SUB
            )

        self.current_mode = "IDLE"
        #self.mode_label.setText("MODE: IDLE")

    # =========================================================
    # E-STOP state machine
    # =========================================================

    def _estop(self):
        print("E-STOP ACTIVATED")

        # If on jog page, return to main first so E-STOP button is visible
        self.stack.setCurrentIndex(PAGE_MAIN)

        self.current_mode = 'ESTOPPED'
        #self.mode_label.setText(f"MODE: {self.current_mode}")

        # Stop any jog command in flight
        self.ros.send_jog('stop')

        self._kill_robot_stack()
        self._kill_manual_init()
        self._kill_encoder_teleop()

        try:
            self.ros.shutdown()
        except Exception as e:
            print(f"ROS shutdown error: {e}")

        self._reset_all_ui()

        self.estop_btn.setText("⛔ HOLD 3s TO RESET")
        self.estop_btn.setStyleSheet("""
            QPushButton {
                background-color: #cc0000;
                color: white;
                border: 3px solid #ff0000;
                border-radius: 12px;
                font-size: 18px;
            }
        """)
        self.estop_latched = True
        self.estop_reset_ready = True

    def _start_estop_hold(self):
        if not self.estop_latched or not self.estop_reset_ready:
            return
        self.estop_hold_time = 0
        self.estop_hold_timer.stop()
        self.estop_hold_timer.start()

    def _stop_estop_hold(self):
        if not self.estop_latched:
            return
        self.estop_hold_timer.stop()
        self.estop_hold_time = 0
        if self.estop_latched:
            self.estop_btn.setText("⛔ HOLD 3s TO RESET")

    def _update_estop_hold(self):
        if not self.estop_latched or not self.estop_reset_ready:
            self.estop_hold_timer.stop()
            return

        self.estop_hold_time += 50
        progress = min(self.estop_hold_time / self.estop_hold_required, 1.0)

        bar = int(progress * 20)
        self.estop_btn.setText(
            "⛔ RESETTING [" + "*" * bar + "-" * (20 - bar) + "]"
        )
        self.estop_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: rgba(200, 0, 0, {0.5 + 0.5 * progress});
                color: white;
                border: 3px solid #ff0000;
                border-radius: 12px;
                font-size: 16px;
            }}
        """)

        if self.estop_hold_time >= self.estop_hold_required:
            self._release_estop()

    def _release_estop(self):
        print("E-STOP RELEASED")
        self.current_mode = 'ESTOP RELEASED'
        #self.mode_label.setText(f"MODE: {self.current_mode}")
        self.estop_latched = False
        self.estop_reset_ready = False
        self.estop_hold_timer.stop()
        self.estop_hold_time = 0
        self._setup_estop_button()
        print("System fully restored and reusable")

    def _setup_estop_button(self):
        """Always restores E-STOP to its default functional state."""
        try:
            self.estop_btn.clicked.disconnect()
        except Exception:
            pass
        try:
            self.estop_btn.pressed.disconnect()
        except Exception:
            pass
        try:
            self.estop_btn.released.disconnect()
        except Exception:
            pass

        self.estop_btn.clicked.connect(self._estop)
        self.estop_btn.pressed.connect(self._start_estop_hold)
        self.estop_btn.released.connect(self._stop_estop_hold)

        self._reset_estop_style()
        self.estop_btn.setText("⛔ E-STOP")
        self.estop_btn.setDown(False)
        self.estop_btn.setChecked(False)
        self.estop_btn.blockSignals(False)

    def _reset_estop_style(self):
        self.estop_btn.setStyleSheet("""
            QPushButton {
                background-color: #2a0000;
                color: #ff4444;
                border: 2px solid #cc0000;
                border-radius: 12px;
            }
            QPushButton:pressed {
                background-color: #440000;
            }
        """)

    # =========================================================
    # UI reset after E-STOP
    # =========================================================

    def _reset_all_ui(self):
        """Restore all UI elements and internal states after E-STOP."""

        for key, btn in self.mode_buttons.items():
            btn.set_active(False)
        self.current_mode = "IDLE"
        #self.mode_label.setText(f"MODE: {self.current_mode}")

        self.encoder_running = False
        self.encoder_processes = []
        if self.encoder_btn:
            self._reset_mode_button(
                self.encoder_btn,
                ENCODER_LABEL,
                ENCODER_SUB
            )

        self.manual_init_running = False
        self.manual_init_process = None
        
        if self.home_btn:
            self._reset_mode_button(
                self.home_btn,
                HOME_LABEL,
                HOME_SUB
            )

        if self.enable_robot_btn:
            self._reset_mode_button(
                self.enable_robot_btn,
                ENABLE_LABEL,
                ENABLE_SUB
            )

    # =========================================================
    # Window close
    # =========================================================

    def closeEvent(self, event):
        print("Closing HMI...")
        self.ros.send_jog('stop')
        self._kill_robot_stack()
        self._kill_manual_init()
        self._kill_encoder_teleop()
        try:
            self.ros.shutdown()
        except Exception as e:
            print(f"ROS shutdown error: {e}")
        event.accept()
