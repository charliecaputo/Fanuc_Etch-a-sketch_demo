#!/usr/bin/env python3

# ─────────────────────────────────────────────
# window.py
# Main HMI window — layout, button logic, and
# page navigation. Process/subprocess handling
# lives in process_manager.py, E-STOP state
# machine lives in estop_controller.py, and QSS
# strings live in styles.py.
# ─────────────────────────────────────────────

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QStackedWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel,
    QPlainTextEdit,
)
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QFont

import styles
from ros_bridge import ROSBridge
from widgets import ModeButton
from jog_page import JogPage
from joint_page import JointPage
from process_manager import ProcessManager
from estop_controller import EstopController

# Page indices for QStackedWidget
PAGE_MAIN = 0
PAGE_JOG = 1
PAGE_JOINT = 2

HOME_LABEL = "Home Robot"
HOME_SUB = "Return to home pose"

ENCODER_LABEL = "Encoder Teleop"
ENCODER_SUB = "XY workspace control"

ENABLE_LABEL = "Enable Robot"
ENABLE_SUB = "Start MoveIt stack"


class HMIWindow(QMainWindow):

    def __init__(self, ros: ROSBridge):
        super().__init__()

        self.ros = ros
        self.current_mode = 'IDLE'
        self.mode_buttons = {}
        self.home_btn = None
        self.encoder_btn = None
        self.enable_robot_btn = None
        self.manual_btn = None

        self.procs = ProcessManager(log_fn=self._print)

        self.setWindowTitle("CRX-10iA Control")
        self.setFixedSize(1024, 600)
        self.show()

        self._build_ui()

        # E-STOP is wired up after the UI exists, since it needs
        # self.estop_btn and a handful of callbacks into this window.
        self.estop = EstopController(
            estop_btn=self.estop_btn,
            log_fn=self._print,
            trigger_fn=self._on_estop_trigger,
            kill_fn=self._on_estop_kill,
            reset_ui_fn=self._reset_all_ui,
            set_mode_fn=self._set_mode,
        )

        ros.mode_changed.connect(self._on_mode_changed)

    # =========================================================
    # Logging
    # =========================================================

    def _log(self, msg: str):
        # Ignore known noisy warnings
        if "overrun" in msg.lower():
            return
        self.term_output.appendPlainText(msg)
        self.term_output.ensureCursorVisible()

    def _print(self, *args, sep=" ", end="\n"):
        msg = sep.join(str(a) for a in args)
        self._log(msg)

    def _set_mode(self, mode: str):
        self.current_mode = mode

    # =========================================================
    # UI Construction
    # =========================================================

    def _build_ui(self):
        # ── Root container ──
        root_widget = QWidget()
        root_widget.setStyleSheet(styles.ROOT_BACKGROUND)
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

        # Page 2 — joint jog screen
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

        header = QLabel("Etch-a-Sketch Demo")
        header.setFont(QFont("Arial", 20))
        header.setStyleSheet(styles.HEADER_LABEL)

        min_btn = QPushButton("—")
        min_btn.setFixedSize(40, 40)
        min_btn.setStyleSheet(styles.MIN_BUTTON)
        min_btn.clicked.connect(self.showMinimized)

        close_btn = QPushButton("✕")
        close_btn.setFixedSize(40, 40)
        close_btn.setStyleSheet(styles.CLOSE_BUTTON)
        close_btn.clicked.connect(self.close)

        header_row.addWidget(header)
        header_row.addStretch()
        header_row.addWidget(min_btn)
        header_row.addWidget(close_btn)

        return header_row

    def _build_status_bar(self):
        self.status_bar = QWidget()
        self.status_bar.setStyleSheet(styles.STATUS_BAR)

        status_layout = QVBoxLayout(self.status_bar)
        status_layout.setContentsMargins(16, 10, 16, 10)

        self.term_output = QPlainTextEdit()
        self.term_output.setReadOnly(True)
        self.term_output.setFont(QFont("Consolas", 12))
        self.term_output.setStyleSheet(styles.TERM_OUTPUT)
        self.term_output.setFixedHeight(80)
        self.term_output.setMaximumBlockCount(10)

        status_layout.addWidget(self.term_output)

        return self.status_bar

    def _build_mode_grid(self):
        grid = QGridLayout()
        grid.setSpacing(12)

        callbacks = {
            "ENABLE_ROBOT": self._enable_robot,
            "HOME": self._run_manual_init,
            "ENCODER_TELEOP": self._run_encoder_teleop,
            "SHIP_POSE": self._shipping_pos,
        }

        modes = [
            ("Encoder Teleop", "XY workspace control", "ENCODER_TELEOP", 0, 0),
            ("Home Robot", "Return to home pose", "HOME", 0, 1),
            ("Ship", "Move to shipping position", "SHIP_POSE", 1, 0),
            ("Enable Robot", "Start MoveIt stack", "ENABLE_ROBOT", 1, 1),
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

    def _build_estop_button(self):
        self.estop_btn = QPushButton("E-Stop")
        self.estop_btn.setFont(QFont("Arial", 18, QFont.Weight.Medium))
        self.estop_btn.setMinimumHeight(90)
        # Styling/wiring happens in EstopController, once it's constructed.
        return self.estop_btn

    def _reset_mode_button(self, btn, label, sub):
        btn.label_widget.setText(label)
        btn.sub_widget.setText(sub)
        btn._set_inactive()

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
        for key, btn in self.mode_buttons.items():
            btn.set_active(key == mode)

    # =========================================================
    # Jog listener (used by the Manual Control page)
    # =========================================================

    def _run_jog_listener(self):
        jl = self.procs.jog_listener_process
        if jl and jl.poll() is None:
            return  # already running

        self.procs.kill_manual_init()
        self.procs.kill_encoder_teleop()

        if self.home_btn:
            self._reset_mode_button(self.home_btn, HOME_LABEL, HOME_SUB)
        if self.encoder_btn:
            self._reset_mode_button(self.encoder_btn, ENCODER_LABEL, ENCODER_SUB)

        cmd = "ros2 run test_py jog_listener_node"
        self.procs.start_jog_listener(cmd)

    def _kill_jog_listener(self):
        if not self.procs.jog_listener_process:
            return

        self.procs.kill_jog_listener()
        if self.manual_btn:
            self._reset_mode_button(self.manual_btn, "Manual Control", "MANUAL_JOG")

    # =========================================================
    # Process-backed mode buttons
    # =========================================================

    def _enable_robot(self):
        if self.procs.robot_process is not None and self.procs.robot_process.poll() is None:
            self._print("Robot already running")
            return

        self._print("Launching robot...")
        self.current_mode = 'ENABLING ROBOT'

        self.enable_robot_btn.label_widget.setText("Starting")
        self.enable_robot_btn.sub_widget.setText("Launching MoveIt")

        cmd = (
            "source /opt/ros/jazzy/setup.bash && "
            "source /home/fanuc/fanuc_ws/install/setup.bash && "
            "ros2 launch test_py fac_moveit_test.launch.py use_mock:=true"
        )

        try:
            proc = self.procs.start_robot_stack(cmd)
            self._print(f"Robot launch started PID={proc.pid}")
            self.enable_robot_btn.label_widget.setText("Robot Enabled")
            self.enable_robot_btn.sub_widget.setText("MoveIt Running")
            self.current_mode = 'IDLE'

        except Exception as e:
            self._print(f"Launch failed: {e}")
            self.enable_robot_btn.label_widget.setText("Launch Failed")
            self.enable_robot_btn.sub_widget.setText("Check Terminal")

    def _run_manual_init(self):
        # ── STOP if already running ──
        if self.procs.manual_init_running:
            self._print("Stopping manual init...")
            self.procs.kill_manual_init()
            self._reset_mode_button(self.home_btn, HOME_LABEL, HOME_SUB)
            return

        # ── START ──
        self.procs.kill_encoder_teleop()
        self._kill_jog_listener()

        if self.manual_btn:
            self._reset_mode_button(self.manual_btn, "Manual Control", "MANUAL_JOG")
        if self.encoder_btn:
            self._reset_mode_button(self.encoder_btn, ENCODER_LABEL, ENCODER_SUB)

        self._print("Starting manual init...")
        self.current_mode = 'MANUAL INITIALIZATION'

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
        self.procs.start_manual_init(cmd)

    def _shipping_pos(self):
        self._print("Shipping position requested — stopping active modes...")

        # 1. Stop encoder teleop
        if self.procs.encoder_running:
            self.procs.kill_encoder_teleop()

        if self.encoder_btn:
            self._reset_mode_button(self.encoder_btn, ENCODER_LABEL, ENCODER_SUB)

        # 2. Stop manual init
        if self.procs.manual_init_running:
            self.procs.kill_manual_init()

        if self.home_btn:
            self._reset_mode_button(self.home_btn, HOME_LABEL, HOME_SUB)

        # 3. Reset mode state
        self.current_mode = "SHIP"
        self._print("Publishing shipping position command...")

        # 4. Publish ROS command
        self.ros.publish_ship_pose(True)

        # 5. Optional: auto-reset bool so it's a clean trigger
        QTimer.singleShot(300, lambda: self.ros.publish_ship_pose(False))

    def _run_encoder_teleop(self):
        # ── STOP if running ──
        if self.procs.encoder_running:
            self._print("Stopping encoder teleop...")
            self.procs.kill_encoder_teleop()
            self._reset_mode_button(self.encoder_btn, ENCODER_LABEL, ENCODER_SUB)
            return

        # ── START ──
        self.procs.kill_manual_init()
        self._kill_jog_listener()

        if self.home_btn:
            self._reset_mode_button(self.home_btn, HOME_LABEL, HOME_SUB)
        if self.encoder_btn:
            self._reset_mode_button(self.encoder_btn, ENCODER_LABEL, ENCODER_SUB)

        self._print("Starting encoder teleop...")
        self.current_mode = 'ENCODER TELEOP'

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
        self.procs.start_encoder_teleop(cmd)

    # =========================================================
    # E-STOP callbacks (invoked by EstopController)
    # =========================================================

    def _on_estop_trigger(self):
        # If on jog page, return to main first so E-STOP button is visible
        self.stack.setCurrentIndex(PAGE_MAIN)
        # Stop any jog command in flight
        self.ros.send_jog('stop')

    def _on_estop_kill(self):
        self.procs.kill_manual_init()
        self.procs.kill_encoder_teleop()

    def _reset_all_ui(self):
        """Restore all UI elements after E-STOP."""
        for key, btn in self.mode_buttons.items():
            btn.set_active(False)
        self.current_mode = "IDLE"

        if self.encoder_btn:
            self._reset_mode_button(self.encoder_btn, ENCODER_LABEL, ENCODER_SUB)
        if self.home_btn:
            self._reset_mode_button(self.home_btn, HOME_LABEL, HOME_SUB)
        if self.enable_robot_btn:
            self._reset_mode_button(self.enable_robot_btn, ENABLE_LABEL, ENABLE_SUB)

    # =========================================================
    # Window close
    # =========================================================

    def closeEvent(self, event):
        print("Closing HMI...")
        self.ros.send_jog('stop')
        self.procs.kill_all()
        try:
            self.ros.shutdown()
        except Exception as e:
            self._print(f"ROS shutdown error: {e}")
        event.accept()
