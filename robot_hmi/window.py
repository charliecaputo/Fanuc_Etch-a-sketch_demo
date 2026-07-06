#!/usr/bin/env python3
# ─────────────────────────────────────────────
# window.py
#
# ROS2 + PyQt6 robot HMI main window.
#
# Responsibilities:
#   • Provide main GUI for robot control
#   • Manage stacked UI pages (Home, Jog, Joint)
#   • Launch and monitor ROS2 processes (MoveIt, teleop, servo, etc.)
#   • Handle mode switching (Home, Encoder Teleop, Manual, Ship, Enable)
#   • Interface with ROSBridge for command + state exchange
#   • Integrate jog_listener node lifecycle management
#   • Coordinate safety systems (E-stop, ship pose lockout)
#   • Enable/disable controls based on robot state
#   • Log system output to terminal widget
#
# NOTE:
#   • This UI assumes a running ROS2 environment (Jazzy or compatible)
#   • ProcessManager is responsible for all subprocess lifecycle control
#   • E-stop overrides all robot actions and forces safe UI reset
#   • Jog pages depend on MoveIt Servo being active
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
from freedrive_page import FreedrivePage
from process_manager import ProcessManager
from estop_controller import EstopController

# Page indices for stacked navigation
PAGE_MAIN = 0
PAGE_JOG = 1
PAGE_JOINT = 2
PAGE_FREEDRIVE = 3

# Mode display constants
HOME_LABEL = "Home Robot"
HOME_SUB = "Return to home pose"

ENCODER_LABEL = "Encoder Teleop"
ENCODER_SUB = "XY workspace control"

ENABLE_LABEL = "Enable Robot"
ENABLE_SUB = "Start MoveIt stack"


class HMIWindow(QMainWindow):
    """
    Main application window for robot HMI.

    This class orchestrates:
        • UI pages (stacked widget navigation)
        • Robot process lifecycle
        • Mode selection logic
        • Logging output
        • E-stop integration
    """

    def __init__(self, ros: ROSBridge):
        super().__init__()

        # ROS communication bridge
        self.ros = ros

        # Current logical system mode
        self.current_mode = 'IDLE'

        # Track dynamically created mode buttons
        self.mode_buttons = {}

        # References to key UI buttons
        self.home_btn = None
        self.encoder_btn = None
        self.enable_robot_btn = None
        self.robot_enabled = False
        self.manual_btn = None
        self.ship_btn = None

        # Subscribe to robot ship-state updates
        ros.ship_state_changed.connect(self._on_ship_state_changed)

        # Process manager (launch/kill ROS nodes and scripts)
        self.procs = ProcessManager(log_fn=self._print)

        # Window configuration
        self.setWindowTitle("CRX-10iA Control")
        self.setFixedSize(1024, 600)

        # Show window (can be swapped to fullscreen if needed)
        self.show()
        #self.showFullScreen()

        # Build UI hierarchy
        self._build_ui()
        self._set_controls_enabled(False)

        # =========================================================
        # E-STOP controller (must be created after UI exists)
        # =========================================================
        self.estop = EstopController(
            estop_btn=self.estop_btn,
            log_fn=self._print,
            trigger_fn=self._on_estop_trigger,
            kill_fn=self._on_estop_kill,
            reset_ui_fn=self._reset_all_ui,
            set_mode_fn=self._set_mode,
        )

        # React to ROS mode updates
        ros.mode_changed.connect(self._on_mode_changed)

    # =========================================================
    # Logging
    # =========================================================

    def _log(self, msg: str):
        """Append message to terminal output widget."""
        # Filter noisy warnings
        if "overrun" in msg.lower():
            return

        self.term_output.appendPlainText(msg)
        self.term_output.ensureCursorVisible()

    def _print(self, *args, sep=" ", end="\n"):
        """Print-style wrapper for logging."""
        msg = sep.join(str(a) for a in args)
        self._log(msg)

    def _set_mode(self, mode: str):
        """Update internal mode state."""
        self.current_mode = mode

    # =========================================================
    # UI Construction
    # =========================================================

    def _build_ui(self):
        """Create root layout and stacked pages."""
        root_widget = QWidget()
        root_widget.setStyleSheet(styles.ROOT_BACKGROUND)
        self.setCentralWidget(root_widget)

        root_layout = QVBoxLayout(root_widget)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        # Stacked widget manages page navigation
        self.stack = QStackedWidget()
        root_layout.addWidget(self.stack)

        # Main page
        self.stack.addWidget(self._build_main_page())

        # Jog pages
        self.jog_page = JogPage(self.ros, back_callback=self._show_main)
        self.stack.addWidget(self.jog_page)

        self.joint_page = JointPage(self.ros, back_callback=self._show_main)
        self.stack.addWidget(self.joint_page)

        self.freedrive_page = FreedrivePage(self.ros, back_callback=self._show_main)
        self.stack.addWidget(self.freedrive_page)

        self.stack.setCurrentIndex(PAGE_MAIN)

    def _build_main_page(self):
        """Assemble the main control dashboard page."""
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
        """Top header bar with title and window controls."""
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
        """Terminal/log output area."""
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
        """Create grid of mode selection buttons."""
        grid = QGridLayout()
        grid.setSpacing(12)

        # Actions triggered by specific mode buttons
        callbacks = {
            "ENABLE_ROBOT": self._enable_robot,
            "HOME": self._run_manual_init,
            "ENCODER_TELEOP": self._run_encoder_teleop,
            "SHIP_POSE": self._shipping_pos,
        }

        modes = [
            ("Encoder Teleop", "XY workspace control", "ENCODER_TELEOP", 0, 0),
            ("Home Robot", "Return to home pose", "HOME", 0, 1),
            ("Ship", "Move to or leave shipping position", "SHIP_POSE", 1, 0),
            ("Enable Robot", "Start MoveIt stack", "ENABLE_ROBOT", 1, 1),
        ]

        # Create mode buttons dynamically
        for label, sub, key, row, col in modes:
            btn = ModeButton(label, sub, key)

            if key in callbacks:
                btn.clicked.connect(callbacks[key])

                # Store references to key buttons for later updates
                if key == "ENABLE_ROBOT":
                    self.enable_robot_btn = btn
                elif key == "HOME":
                    self.home_btn = btn
                elif key == "ENCODER_TELEOP":
                    self.encoder_btn = btn
                elif key == "SHIP_POSE":
                    self.ship_btn = btn
            else:
                btn.clicked.connect(
                    lambda checked, k=key: self.ros.send_mode(k)
                )
                self.mode_buttons[key] = btn

            grid.addWidget(btn, row, col)

        # Manual control (spans full width)
        manual_btn = ModeButton("Manual Control", "Cartesian jog XYZ", "MANUAL_JOG")
        manual_btn.clicked.connect(self._show_jog)
        self.manual_btn = manual_btn
        grid.addWidget(manual_btn, 2, 0, 1, 2)

        return grid

    def _build_estop_button(self):
        """Create E-stop button (wired later by controller)."""
        self.estop_btn = QPushButton("E-Stop")
        self.estop_btn.setFont(QFont("Arial", 18, QFont.Weight.Medium))
        self.estop_btn.setMinimumHeight(90)
        return self.estop_btn

    def _reset_mode_button(self, btn, label, sub):
        """Reset a ModeButton back to default inactive state."""
        btn.label_widget.setText(label)
        btn.sub_widget.setText(sub)
        btn._set_inactive()

    # =========================================================
    # Page navigation
    # =========================================================

    def _show_jog(self):
        """Switch to Cartesian jog page."""
        self._run_jog_listener()
        self.stack.setCurrentIndex(PAGE_JOG)

    def _show_main(self, mode=None):
        """Return to main page or switch between jog modes."""
        if mode != "joint" and mode != "cartesian":
            self._kill_jog_listener()

        if mode == "joint":
            self.stack.setCurrentIndex(PAGE_JOINT)
            return

        if mode == "cartesian":
            self.stack.setCurrentIndex(PAGE_JOG)
            return

        if mode == "freedrive":
            self.stack.setCurrentIndex(PAGE_FREEDRIVE)
            return

        self.stack.setCurrentIndex(PAGE_MAIN)

    # =========================================================
    # Mode updates
    # =========================================================

    def _on_mode_changed(self, mode: str):
        """Highlight active mode button."""
        self.current_mode = mode
        for key, btn in self.mode_buttons.items():
            btn.set_active(key == mode)

    # =========================================================
    # Jog process management
    # =========================================================

    def _run_jog_listener(self):
        """Start jog listener process if not already running."""
        jl = self.procs.jog_listener_process
        if jl and jl.poll() is None:
            return

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
    # Mode execution callbacks
    # =========================================================
    # (Enable robot, home, encoder teleop, ship pose)
    # =========================================================

    def _enable_robot(self):
        """Launch MoveIt / robot stack."""
        if self.procs.robot_process and self.procs.robot_process.poll() is None:
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
            self._set_controls_enabled(True)
        except Exception as e:
            self._print(f"Launch failed: {e}")
            self.enable_robot_btn.label_widget.setText("Launch Failed")
            self.enable_robot_btn.sub_widget.setText("Check Terminal")

    def _run_manual_init(self):
        """Run or stop manual initialization routine."""
        if self.ros.in_ship_pose:
            self._print("Cannot home robot while in shipping position.")
            return

        if self.procs.manual_init_running:
            self._print("Stopping manual init...")
            self.procs.kill_manual_init()
            self._reset_mode_button(self.home_btn, HOME_LABEL, HOME_SUB)
            return

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
        """Send robot to shipping pose (and stop other modes)."""
        self._print("Shipping position requested — stopping active modes...")

        if self.procs.encoder_running:
            self.procs.kill_encoder_teleop()

        if self.encoder_btn:
            self._reset_mode_button(self.encoder_btn, ENCODER_LABEL, ENCODER_SUB)

        if self.procs.manual_init_running:
            self.procs.kill_manual_init()

        if self.home_btn:
            self._reset_mode_button(self.home_btn, HOME_LABEL, HOME_SUB)

        self.current_mode = "SHIP"
        self._print("Publishing shipping position command...")

        self.ros.publish_ship_pose(True)
        QTimer.singleShot(300, lambda: self.ros.publish_ship_pose(False))

    def _run_encoder_teleop(self):
        """Start/stop encoder teleop process."""
        if self.ros.in_ship_pose:
            self._print("Cannot start encoder teleop while in shipping position.")
            return

        if self.procs.encoder_running:
            self._print("Stopping encoder teleop...")
            self.procs.kill_encoder_teleop()
            self._reset_mode_button(self.encoder_btn, ENCODER_LABEL, ENCODER_SUB)
            return

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
            "ros2 run test_py encoder_read --ros-args --params-file ~/fanuc_ws/src/test_py/config/demo_params.yaml & "
            "ros2 run test_py servo_control "
        )
        self.procs.start_encoder_teleop(cmd)

    # =========================================================
    # E-STOP integration
    # =========================================================

    def _on_estop_trigger(self):
        """Called when E-stop is pressed."""
        self.stack.setCurrentIndex(PAGE_MAIN)
        self.ros.send_jog('stop')

    def _on_estop_kill(self):
        """Kill all running robot processes."""
        self.procs.kill_manual_init()
        self.procs.kill_encoder_teleop()

    def _reset_all_ui(self):
        """Reset UI after E-stop recovery."""
        for key, btn in self.mode_buttons.items():
            btn.set_active(False)

        self.current_mode = "IDLE"

        if self.encoder_btn:
            self._reset_mode_button(self.encoder_btn, ENCODER_LABEL, ENCODER_SUB)
        if self.home_btn:
            self._reset_mode_button(self.home_btn, HOME_LABEL, HOME_SUB)
        if self.enable_robot_btn:
            self._reset_mode_button(self.enable_robot_btn, ENABLE_LABEL, ENABLE_SUB)

    def _set_controls_enabled(self, enabled: bool):
        """Enable/disable main UI controls depending on robot state."""
        
        # store state
        self.robot_enabled = enabled

        # buttons you want locked/unlocked
        buttons = [
            self.home_btn,
            self.encoder_btn,
            self.manual_btn,
            self.ship_btn
        ]

        for btn in buttons:
            if btn:
                btn.setEnabled(enabled)

        # optionally also disable ship + enable button logic
        if self.enable_robot_btn:
            self.enable_robot_btn.setEnabled(True)  # usually always clickable

        # optional: visually indicate disabled state
        if not enabled:
            self._print("Controls locked until robot is enabled")
        else:
            self._print("Controls enabled")

    def _on_ship_state_changed(self, in_ship_pose: bool):
        """Disable controls when robot is in ship pose."""
        self.home_btn.setEnabled(not in_ship_pose)
        self.encoder_btn.setEnabled(not in_ship_pose)

        if in_ship_pose:
            self._print("Robot is in shipping position.")
        else:
            self._print("Robot has left shipping position.")

    # =========================================================
    # Shutdown
    # =========================================================

    def closeEvent(self, event):
        """Clean shutdown of UI and robot processes."""
        print("Closing HMI...")

        self.ros.send_jog('stop')
        self.procs.kill_all()

        try:
            self.ros.shutdown()
        except Exception as e:
            self._print(f"ROS shutdown error: {e}")

        event.accept()
