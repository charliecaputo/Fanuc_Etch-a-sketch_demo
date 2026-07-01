import sys
import os
from enum import Enum, auto
from PyQt5 import QtWidgets, uic, QtCore


class RobotState(Enum):
    AUTO_IDLE = auto()
    AUTO_RUNNING = auto()
    AUTO_STOPPING = auto()
    AUTO_FAULTED = auto()
    AUTO_ABORTED = auto()
    MANUAL_JOGGING = auto()
    HAND_GUIDE = auto()
    HOMING = auto()
    SHIPPING = auto()


class KarelFileInterface:
    """Lightweight file-based interface between HMI (Python) and KAREL.

    Protocol:
        - HMI appends commands to MD-mirrored file: HMI_TO_KAREL.CMD
          Format: seq|COMMAND|k1=v1,k2=v2
        - KAREL appends/rewrites latest status to: KAREL_TO_HMI.STS
          Format: state=AUTO_IDLE;manual=0;message=ready
    """

    CMD_FILE = "HMI_TO_KAREL.CMD"
    STATUS_FILE = "KAREL_TO_HMI.STS"

    def __init__(self, base_dir: str):
        self.base_dir = base_dir
        self.cmd_path = os.path.join(base_dir, self.CMD_FILE)
        self.status_path = os.path.join(base_dir, self.STATUS_FILE)
        self._seq = 0
        self._last_status_mtime = 0.0
        self._ensure_files()

    def _ensure_files(self):
        if not os.path.exists(self.cmd_path):
            with open(self.cmd_path, "w", encoding="ascii"):
                pass
        if not os.path.exists(self.status_path):
            with open(self.status_path, "w", encoding="ascii") as f:
                f.write("state=AUTO_IDLE;manual=0;message=boot\n")

    def _escape_value(self, value):
        text = str(value)
        for token in ["|", ",", ";", "\n", "\r"]:
            text = text.replace(token, "_")
        return text

    def send(self, command: str, **fields):
        self._seq += 1
        payload = ",".join(
            f"{key}={self._escape_value(value)}" for key, value in fields.items()
        )
        line = f"{self._seq}|{command}|{payload}\n"
        with open(self.cmd_path, "a", encoding="ascii") as f:
            f.write(line)

    def poll_status(self):
        if not os.path.exists(self.status_path):
            return None

        mtime = os.path.getmtime(self.status_path)
        if mtime <= self._last_status_mtime:
            return None
        self._last_status_mtime = mtime

        with open(self.status_path, "r", encoding="ascii", errors="ignore") as f:
            lines = [line.strip() for line in f.readlines() if line.strip()]

        if not lines:
            return None

        status = {}
        for token in lines[-1].split(";"):
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            status[key.strip().lower()] = value.strip()
        return status


class RobotHMI(QtWidgets.QMainWindow):
    """Simple HMI shell for a robot application.

        Main screen:
            - Start / Stop / Reset / Abort
            - Manual button goes to Manual page

        Manual page:
            - Manual mode toggle, JOGGING/HAND_GUIDE/HOMING/SHIPPING controls, speed override
            - Navigation to dedicated Jog page
            - Exit Application button (Manual Mode only; 2 second hold)

        UI is split across MainPage.ui, ManualPage.ui, and JogPage.ui and loaded into one stacked view.
    """

    HOLD_TO_EXIT_MS = 2000
    AXIS_KEYS = ["x", "y", "z", "w", "p", "r"]
    JOG_AXES = {
        "cartesian": ["X", "Y", "Z", "W", "P", "R"],
        "joint": ["J1", "J2", "J3", "J4", "J5", "J6"],
        "simple": ["X", "Y", "Z", "W", "P", "R"],
    }
    JOG_SIMPLE_MAP = {
        "left": ("x", -1),
        "right": ("x", 1),
        "front": ("y", 1),
        "back": ("y", -1),
        "up": ("z", 1),
        "down": ("z", -1),
    }

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Robot HMI")
        #self.setFixedSize(800, 480)
        self.setFixedSize(1024, 600)
        self._load_split_pages()

        # --- State ---
        self.state = RobotState.AUTO_IDLE
        self.manual_mode = False
        self.speed_override = 100
        self.jog_mode = "cartesian"
        self._active_jogs = set()
        self._jog_page_first_visit = True
        self.karel_if = None

        # --- Widgets (by objectName across split .ui pages) ---
        self.stacked = self.findChild(QtWidgets.QStackedWidget, "stackedWidget")

        self.btn_start = self.findChild(QtWidgets.QPushButton, "btn_start")
        self.btn_stop = self.findChild(QtWidgets.QPushButton, "btn_stop")
        self.btn_reset = self.findChild(QtWidgets.QPushButton, "btn_reset")
        self.btn_abort = self.findChild(QtWidgets.QPushButton, "btn_abort")
        self.btn_gear = self.findChild(QtWidgets.QPushButton, "btn_gear")
        self.lbl_status = self.findChild(QtWidgets.QLabel, "lbl_status")
        self.lbl_jog_status = self.findChild(QtWidgets.QLabel, "lbl_jog_status")
        self.txt_log = self.findChild(QtWidgets.QTextEdit, "txt_log")

        self.btn_back = self.findChild(QtWidgets.QPushButton, "btn_back")
        self.btn_jog_back = self.findChild(QtWidgets.QPushButton, "btn_jog_back")
        self.btn_toggle_manual = self.findChild(QtWidgets.QPushButton, "btn_toggle_manual")
        self.btn_kill_app = self.findChild(QtWidgets.QPushButton, "btn_kill_app")

        self.sld_speed = self.findChild(QtWidgets.QSlider, "sld_speed")
        self.spn_speed = self.findChild(QtWidgets.QSpinBox, "spn_speed")
        self.sld_speed_main = self.findChild(QtWidgets.QSlider, "sld_speed_main")
        self.spn_speed_main = self.findChild(QtWidgets.QSpinBox, "spn_speed_main")
        self.sld_speed_jog = self.findChild(QtWidgets.QSlider, "sld_speed_jog")
        self.spn_speed_jog = self.findChild(QtWidgets.QSpinBox, "spn_speed_jog")
        self.lbl_maint_status = self.findChild(QtWidgets.QLabel, "lbl_maint_status")
        self.txt_maint_log = self.findChild(QtWidgets.QTextEdit, "txt_maint_log")

        self.btn_mode_teach = self.findChild(QtWidgets.QPushButton, "btn_mode_teach")
        self.btn_mode_handguide = self.findChild(QtWidgets.QPushButton, "btn_mode_handguide")
        self.btn_move_home = self.findChild(QtWidgets.QPushButton, "btn_move_home")
        self.btn_move_shipping = self.findChild(QtWidgets.QPushButton, "btn_move_shipping")
        self.btn_manual_reset = self.findChild(QtWidgets.QPushButton, "btn_manual_reset")

        self.btn_jog_x_minus = self.findChild(QtWidgets.QPushButton, "btn_jog_x_minus")
        self.btn_jog_x_plus = self.findChild(QtWidgets.QPushButton, "btn_jog_x_plus")
        self.btn_jog_y_minus = self.findChild(QtWidgets.QPushButton, "btn_jog_y_minus")
        self.btn_jog_y_plus = self.findChild(QtWidgets.QPushButton, "btn_jog_y_plus")
        self.btn_jog_z_minus = self.findChild(QtWidgets.QPushButton, "btn_jog_z_minus")
        self.btn_jog_z_plus = self.findChild(QtWidgets.QPushButton, "btn_jog_z_plus")
        self.btn_jog_w_minus = self.findChild(QtWidgets.QPushButton, "btn_jog_w_minus")
        self.btn_jog_w_plus = self.findChild(QtWidgets.QPushButton, "btn_jog_w_plus")
        self.btn_jog_p_minus = self.findChild(QtWidgets.QPushButton, "btn_jog_p_minus")
        self.btn_jog_p_plus = self.findChild(QtWidgets.QPushButton, "btn_jog_p_plus")
        self.btn_jog_r_minus = self.findChild(QtWidgets.QPushButton, "btn_jog_r_minus")
        self.btn_jog_r_plus = self.findChild(QtWidgets.QPushButton, "btn_jog_r_plus")

        self.lbl_axis_x = self.findChild(QtWidgets.QLabel, "lbl_axis_x")
        self.lbl_axis_y = self.findChild(QtWidgets.QLabel, "lbl_axis_y")
        self.lbl_axis_z = self.findChild(QtWidgets.QLabel, "lbl_axis_z")
        self.lbl_axis_w = self.findChild(QtWidgets.QLabel, "lbl_axis_w")
        self.lbl_axis_p = self.findChild(QtWidgets.QLabel, "lbl_axis_p")
        self.lbl_axis_r = self.findChild(QtWidgets.QLabel, "lbl_axis_r")
        self.grp_jog = self.findChild(QtWidgets.QGroupBox, "grp_jog")
        self.grp_simple_jog = self.findChild(QtWidgets.QGroupBox, "grp_simple_jog")

        self.rdo_jog_cartesian = self.findChild(QtWidgets.QRadioButton, "rdo_jog_cartesian")
        self.rdo_jog_joint = self.findChild(QtWidgets.QRadioButton, "rdo_jog_joint")
        self.rdo_jog_simple = self.findChild(QtWidgets.QRadioButton, "rdo_jog_simple")

        self.btn_simple_up = self.findChild(QtWidgets.QPushButton, "btn_simple_up")
        self.btn_simple_down = self.findChild(QtWidgets.QPushButton, "btn_simple_down")
        self.btn_simple_left = self.findChild(QtWidgets.QPushButton, "btn_simple_left")
        self.btn_simple_right = self.findChild(QtWidgets.QPushButton, "btn_simple_right")
        self.btn_simple_front = self.findChild(QtWidgets.QPushButton, "btn_simple_front")
        self.btn_simple_back = self.findChild(QtWidgets.QPushButton, "btn_simple_back")
        self.btn_simple_home = self.findChild(QtWidgets.QPushButton, "btn_simple_home")

        # --- KAREL interface ---
        self.karel_if = KarelFileInterface(os.path.dirname(__file__))

        # --- Hold-to-exit timer ---
        self._exit_hold_timer = QtCore.QTimer(self)
        self._exit_hold_timer.setSingleShot(True)
        self._exit_hold_timer.setInterval(self.HOLD_TO_EXIT_MS)
        self._exit_hold_timer.timeout.connect(self._exit_application_now)

        self._karel_status_timer = QtCore.QTimer(self)
        self._karel_status_timer.setInterval(250)
        self._karel_status_timer.timeout.connect(self._poll_karel_status)
        self._karel_status_timer.start()

        # --- Signal wiring ---
        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop.clicked.connect(self.on_stop)
        self.btn_reset.clicked.connect(self.on_reset)
        self.btn_abort.clicked.connect(self.on_abort)
        self.btn_gear.clicked.connect(self.goto_maintenance)

        self.btn_back.clicked.connect(self.goto_main)
        if self.btn_jog_back is not None:
            self.btn_jog_back.clicked.connect(self.goto_maintenance)
        self.btn_toggle_manual.clicked.connect(self.on_toggle_manual)

        if self.btn_mode_teach is not None:
            self.btn_mode_teach.clicked.connect(self.on_set_teach)
        if self.btn_mode_handguide is not None:
            self.btn_mode_handguide.clicked.connect(self.on_set_handguide)
        if self.btn_move_home is not None:
            self.btn_move_home.clicked.connect(self.on_move_to_home)
        if self.btn_move_shipping is not None:
            self.btn_move_shipping.clicked.connect(self.on_move_to_shipping)
        if self.btn_manual_reset is not None:
            self.btn_manual_reset.clicked.connect(self.on_manual_reset)

        if self.rdo_jog_cartesian is not None:
            self.rdo_jog_cartesian.toggled.connect(lambda checked: self.on_jog_mode_changed("cartesian", checked))
        if self.rdo_jog_joint is not None:
            self.rdo_jog_joint.toggled.connect(lambda checked: self.on_jog_mode_changed("joint", checked))
        if self.rdo_jog_simple is not None:
            self.rdo_jog_simple.toggled.connect(lambda checked: self.on_jog_mode_changed("simple", checked))

        self._wire_jog_button(self.btn_jog_x_minus, "x", -1)
        self._wire_jog_button(self.btn_jog_x_plus, "x", 1)
        self._wire_jog_button(self.btn_jog_y_minus, "y", -1)
        self._wire_jog_button(self.btn_jog_y_plus, "y", 1)
        self._wire_jog_button(self.btn_jog_z_minus, "z", -1)
        self._wire_jog_button(self.btn_jog_z_plus, "z", 1)
        self._wire_jog_button(self.btn_jog_w_minus, "w", -1)
        self._wire_jog_button(self.btn_jog_w_plus, "w", 1)
        self._wire_jog_button(self.btn_jog_p_minus, "p", -1)
        self._wire_jog_button(self.btn_jog_p_plus, "p", 1)
        self._wire_jog_button(self.btn_jog_r_minus, "r", -1)
        self._wire_jog_button(self.btn_jog_r_plus, "r", 1)

        self._wire_simple_button(self.btn_simple_left, "left")
        self._wire_simple_button(self.btn_simple_right, "right")
        self._wire_simple_button(self.btn_simple_front, "front")
        self._wire_simple_button(self.btn_simple_back, "back")
        self._wire_simple_button(self.btn_simple_up, "up")
        self._wire_simple_button(self.btn_simple_down, "down")
        if self.btn_simple_home is not None:
            self.btn_simple_home.clicked.connect(self.on_move_to_home)

        if self.sld_speed is not None:
            self.sld_speed.valueChanged.connect(lambda value: self.on_speed_slider(value, self.sld_speed))
        if self.spn_speed is not None:
            self.spn_speed.valueChanged.connect(lambda value: self.on_speed_spin(value, self.spn_speed))
        if self.sld_speed_main is not None:
            self.sld_speed_main.valueChanged.connect(lambda value: self.on_speed_slider(value, self.sld_speed_main))
        if self.spn_speed_main is not None:
            self.spn_speed_main.valueChanged.connect(lambda value: self.on_speed_spin(value, self.spn_speed_main))
        if self.sld_speed_jog is not None:
            self.sld_speed_jog.valueChanged.connect(lambda value: self.on_speed_slider(value, self.sld_speed_jog))
        if self.spn_speed_jog is not None:
            self.spn_speed_jog.valueChanged.connect(lambda value: self.on_speed_spin(value, self.spn_speed_jog))

        # Press-and-hold behavior for Exit button
        if self.btn_kill_app is not None:
            self._kill_btn_default_text = self.btn_kill_app.text()
            self.btn_kill_app.pressed.connect(self.on_exit_press)
            self.btn_kill_app.released.connect(self.on_exit_release)

        # Keep logs read-only if present
        if self.txt_log is not None:
            self.txt_log.setReadOnly(True)
        if self.txt_maint_log is not None:
            self.txt_maint_log.setReadOnly(True)

        # Initialize UI values
        self._sync_speed_widgets()

        self._log("HMI loaded")
        self._update_jog_mode_labels()
        self._update_status()
        self._update_ui_enabled()

    # ----------------------
    # Navigation
    # ----------------------
    def goto_maintenance(self):
        if self.stacked is not None:
            self.stacked.setCurrentIndex(1)
        self._log("Entered Manual screen")
        self._update_status()
        self._update_ui_enabled()

    def goto_main(self):
        if self.stacked is not None:
            self.stacked.setCurrentIndex(0)
        self._log("Returned to Main screen")
        self._update_status()
        self._update_ui_enabled()

    def goto_jog(self):
        if self._jog_page_first_visit:
            self.jog_mode = "simple"
            if self.rdo_jog_simple is not None:
                self.rdo_jog_simple.setChecked(True)
            self._update_jog_mode_labels()
            self._jog_page_first_visit = False
        if self.stacked is not None:
            self.stacked.setCurrentIndex(2)
        self._log("Entered Jog page")
        self._update_status()
        self._update_ui_enabled()

    def _resolve_ui_path(self, names):
        base_dir = os.path.dirname(__file__)
        candidates = [os.path.join(base_dir, name) for name in names]
        ui_path = next((p for p in candidates if os.path.exists(p)), None)
        if ui_path:
            return ui_path
        raise FileNotFoundError("UI file not found. Expected one of: " + ", ".join(names))

    def _load_split_pages(self):
        main_ui_path = self._resolve_ui_path(["MainPage.ui", "mainpage.ui", "Main.ui", "main.ui"])
        manual_ui_path = self._resolve_ui_path(["ManualPage.ui", "manualpage.ui", "Manual.ui", "manual.ui"])
        jog_ui_path = self._resolve_ui_path(["JogPage.ui", "jogpage.ui", "Jog.ui", "jog.ui"])

        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)

        root_layout = QtWidgets.QVBoxLayout(central)
        root_layout.setContentsMargins(0, 0, 0, 0)

        self.stacked = QtWidgets.QStackedWidget(central)
        self.stacked.setObjectName("stackedWidget")
        root_layout.addWidget(self.stacked)

        self.page_main = uic.loadUi(main_ui_path)
        self.page_manual = uic.loadUi(manual_ui_path)
        self.page_jog = uic.loadUi(jog_ui_path)

        self.stacked.addWidget(self.page_main)
        self.stacked.addWidget(self.page_manual)
        self.stacked.addWidget(self.page_jog)

    # ----------------------
    # Main actions
    # ----------------------
    def on_start(self):
        # Manual mode uses Start as a JOGGING -> HAND_GUIDE transition.
        if self.manual_mode:
            if self.state == RobotState.MANUAL_JOGGING:
                self.state = RobotState.HAND_GUIDE
                self._log("START pressed: JOGGING -> HAND_GUIDE")
            elif self.state == RobotState.HAND_GUIDE:
                self._log("Start ignored: already HAND_GUIDE")
            else:
                self.state = RobotState.MANUAL_JOGGING
                self._log("START pressed: entered JOGGING")
            self._update_status()
            self._update_ui_enabled()
            self._send_karel("START", mode="MANUAL", state=self.state.name, speed=self.speed_override)
            return

        if self.state == RobotState.AUTO_FAULTED:
            self._log("Start blocked: FAULTED active")
            self._toast("Cannot Start", "Reset faults before starting.")
            return
        if self.state == RobotState.AUTO_RUNNING:
            self._log("Start ignored: already RUNNING")
            return

        self.state = RobotState.AUTO_RUNNING
        self._log(f"START pressed (Speed={self.speed_override}%, Manual={self.manual_mode})")
        self._update_status()
        self._update_ui_enabled()
        self._send_karel("START", mode="AUTO", state=self.state.name, speed=self.speed_override)

    def on_stop(self):
        # Manual mode uses Stop as a HAND_GUIDE -> JOGGING transition.
        if self.manual_mode:
            if self.state == RobotState.HAND_GUIDE:
                self.state = RobotState.MANUAL_JOGGING
                self._log("STOP pressed: HAND_GUIDE -> JOGGING")
            else:
                self._log("Stop ignored: manual mode not in HAND_GUIDE")
                return
        else:
            if self.state != RobotState.AUTO_RUNNING:
                self._log("Stop ignored: not RUNNING")
                return

            self.state = RobotState.AUTO_STOPPING
            self._log("STOP pressed")
        self._update_status()
        self._update_ui_enabled()
        self._send_karel("STOP", state=self.state.name)

    def on_reset(self):
        prev = self.state
        self.state = RobotState.MANUAL_JOGGING if self.manual_mode else RobotState.AUTO_IDLE
        self._log(f"RESET pressed (from {prev.name})")
        self._update_status()
        self._update_ui_enabled()
        self._send_karel("RESET", state=self.state.name)

    def on_abort(self):
        self.state = RobotState.AUTO_ABORTED if not self.manual_mode else RobotState.MANUAL_JOGGING
        self._log("ABORT pressed")
        self._update_status()
        self._update_ui_enabled()
        self._send_karel("ABORT", state=self.state.name)

    # ----------------------
    # Manual page actions
    # ----------------------
    def on_toggle_manual(self):
        self.manual_mode = not self.manual_mode
        self._log(f"Manual mode {'ENABLED' if self.manual_mode else 'DISABLED'}")

        # Keep state in the proper mode bucket when toggling modes.
        if self.manual_mode:
            if self.state in {
                RobotState.AUTO_IDLE,
                RobotState.AUTO_RUNNING,
                RobotState.AUTO_STOPPING,
                RobotState.AUTO_FAULTED,
                RobotState.AUTO_ABORTED,
            }:
                self.state = RobotState.MANUAL_JOGGING
        else:
            if self.state in {
                RobotState.MANUAL_JOGGING,
                RobotState.HAND_GUIDE,
                RobotState.HOMING,
                RobotState.SHIPPING,
            }:
                self.state = RobotState.AUTO_IDLE

        # If manual mode is turned off mid-hold, cancel exit hold.
        if not self.manual_mode:
            self._stop_all_jogs("manual mode disabled")
            self._cancel_exit_hold(reset_text=True)

        self._update_status()
        self._update_ui_enabled()
        self._send_karel("SET_MANUAL", enabled=int(self.manual_mode), state=self.state.name)

    def on_set_teach(self):
        if not self.manual_mode:
            self._toast("Manual Mode Required", "Enable Manual Mode first.")
            return
        if self.state != RobotState.MANUAL_JOGGING:
            self._stop_all_jogs("switching to JOGGING")
            self.state = RobotState.MANUAL_JOGGING
            self._log("Manual state set: JOGGING")
            self._send_karel("SET_STATE", state=self.state.name)
        self.goto_jog()

    def on_set_handguide(self):
        if not self.manual_mode:
            self._toast("Manual Mode Required", "Enable Manual Mode first.")
            return
        if self.state != RobotState.HAND_GUIDE:
            self._stop_all_jogs("switching to HAND_GUIDE")
            self.state = RobotState.HAND_GUIDE
            self._log("Manual state set: HAND_GUIDE")
            self._update_status()
            self._update_ui_enabled()
            self._send_karel("SET_STATE", state=self.state.name)

    def on_jog_mode_changed(self, mode: str, checked: bool):
        if not checked:
            return
        if self.jog_mode == mode:
            if self.manual_mode and self.state != RobotState.MANUAL_JOGGING:
                self.state = RobotState.MANUAL_JOGGING
                self._log("Manual state set: JOGGING (via jog mode toggle)")
                self._update_status()
                self._update_ui_enabled()
            return
        self._stop_all_jogs("jog mode changed")
        self.jog_mode = mode
        if self.manual_mode and self.state != RobotState.MANUAL_JOGGING:
            self.state = RobotState.MANUAL_JOGGING
            self._log("Manual state set: JOGGING (via jog mode toggle)")
        self._update_jog_mode_labels()
        self._log(f"Jog mode set: {mode.upper()}")
        self._update_status()
        self._update_ui_enabled()
        self._send_karel("SET_JOG_MODE", mode=mode)

    def on_move_to_home(self):
        if not self.manual_mode:
            self._toast("Manual Mode Required", "Enable Manual Mode first.")
            return
        self._stop_all_jogs("MoveTo HOME")
        self.state = RobotState.HOMING
        self._log("MoveTo command: HOME")
        self._update_status()
        self._update_ui_enabled()
        self._send_karel("MOVE_TO", target="HOME", state=self.state.name)

    def on_move_to_shipping(self):
        if not self.manual_mode:
            self._toast("Manual Mode Required", "Enable Manual Mode first.")
            return
        self._stop_all_jogs("MoveTo SHIPPING")
        self.state = RobotState.SHIPPING
        self._log("MoveTo command: SHIPPING")
        self._update_status()
        self._update_ui_enabled()
        self._send_karel("MOVE_TO", target="SHIPPING", state=self.state.name)

    def on_manual_reset(self):
        self.on_reset()

    def _wire_jog_button(self, button, axis_key: str, direction: int):
        if button is None:
            return
        button.pressed.connect(lambda a=axis_key, d=direction: self.on_jog_press(a, d))
        button.released.connect(lambda a=axis_key, d=direction: self.on_jog_release(a, d))

    def _wire_simple_button(self, button, direction_name: str):
        if button is None:
            return
        axis_key, direction = self.JOG_SIMPLE_MAP[direction_name]
        button.pressed.connect(lambda a=axis_key, d=direction: self.on_jog_press(a, d))
        button.released.connect(lambda a=axis_key, d=direction: self.on_jog_release(a, d))

    def _axis_name(self, axis_key: str) -> str:
        idx = self.AXIS_KEYS.index(axis_key)
        return self.JOG_AXES[self.jog_mode][idx]

    def on_jog_press(self, axis_key: str, direction: int):
        if not self.manual_mode:
            self._toast("Manual Mode Required", "Enable Manual Mode to jog axes.")
            return
        if self.state != RobotState.MANUAL_JOGGING:
            self._toast("JOGGING Mode Required", "Set state to JOGGING for jog operation.")
            return

        jog_key = (axis_key, direction)
        if jog_key in self._active_jogs:
            return
        self._active_jogs.add(jog_key)

        axis = self._axis_name(axis_key)
        sign = "+" if direction > 0 else "-"
        self._log(f"JOG START {axis}{sign} at {self.speed_override}%")
        self._send_karel(
            "JOG_START",
            mode=self.jog_mode,
            axis=axis,
            direction=sign,
            speed=self.speed_override,
        )

    def on_jog_release(self, axis_key: str, direction: int):
        jog_key = (axis_key, direction)
        if jog_key not in self._active_jogs:
            return
        self._active_jogs.remove(jog_key)

        axis = self._axis_name(axis_key)
        sign = "+" if direction > 0 else "-"
        self._log(f"JOG STOP  {axis}{sign}")
        self._send_karel("JOG_STOP", mode=self.jog_mode, axis=axis, direction=sign)

    def _stop_all_jogs(self, reason: str):
        if not self._active_jogs:
            return
        active = sorted(self._active_jogs)
        self._active_jogs.clear()
        tokens = []
        for axis_key, direction in active:
            axis = self._axis_name(axis_key)
            sign = "+" if direction > 0 else "-"
            tokens.append(f"{axis}{sign}")
        self._log(f"JOG STOP ALL ({reason}): {', '.join(tokens)}")

    def _update_jog_mode_labels(self):
        is_simple = self.jog_mode == "simple"
        if self.grp_jog is not None:
            self.grp_jog.setVisible(not is_simple)
            self.grp_jog.setTitle(f"Press And Hold Jog ({self.jog_mode.upper()})")
        if self.grp_simple_jog is not None:
            self.grp_simple_jog.setVisible(is_simple)
            self.grp_simple_jog.setTitle("Press And Hold Simple Jog")

        if is_simple:
            return

        labels = self.JOG_AXES[self.jog_mode]
        label_widgets = [
            self.lbl_axis_x,
            self.lbl_axis_y,
            self.lbl_axis_z,
            self.lbl_axis_w,
            self.lbl_axis_p,
            self.lbl_axis_r,
        ]
        minus_buttons = [
            self.btn_jog_x_minus,
            self.btn_jog_y_minus,
            self.btn_jog_z_minus,
            self.btn_jog_w_minus,
            self.btn_jog_p_minus,
            self.btn_jog_r_minus,
        ]
        plus_buttons = [
            self.btn_jog_x_plus,
            self.btn_jog_y_plus,
            self.btn_jog_z_plus,
            self.btn_jog_w_plus,
            self.btn_jog_p_plus,
            self.btn_jog_r_plus,
        ]

        for i, axis in enumerate(labels):
            if label_widgets[i] is not None:
                label_widgets[i].setText(axis)
            if minus_buttons[i] is not None:
                minus_buttons[i].setText(f"{axis}-")
            if plus_buttons[i] is not None:
                plus_buttons[i].setText(f"{axis}+")

    def _all_speed_sliders(self):
        return [
            self.sld_speed,
            self.sld_speed_main,
            self.sld_speed_jog,
        ]

    def _all_speed_spins(self):
        return [
            self.spn_speed,
            self.spn_speed_main,
            self.spn_speed_jog,
        ]

    def _sync_speed_widgets(self, source_widget=None):
        for slider in self._all_speed_sliders():
            if slider is None or slider is source_widget:
                continue
            if slider.value() != self.speed_override:
                slider.blockSignals(True)
                slider.setValue(self.speed_override)
                slider.blockSignals(False)

        for spin in self._all_speed_spins():
            if spin is None or spin is source_widget:
                continue
            if spin.value() != self.speed_override:
                spin.blockSignals(True)
                spin.setValue(self.speed_override)
                spin.blockSignals(False)

    def on_speed_slider(self, value: int, source_widget=None):
        self.speed_override = value
        self._sync_speed_widgets(source_widget=source_widget)
        self._update_status(lite=True)
        self._send_karel("SET_SPEED", speed=value)

    def on_speed_spin(self, value: int, source_widget=None):
        self.speed_override = value
        self._sync_speed_widgets(source_widget=source_widget)
        self._update_status(lite=True)
        self._send_karel("SET_SPEED", speed=value)

    # ----------------------
    # Exit Application (Manual Mode only + 2s hold)
    # ----------------------
    def on_exit_press(self):
        if not self.manual_mode:
            return
        if self.btn_kill_app is not None:
            self.btn_kill_app.setText("HOLD 2s TO EXIT…")
        self._exit_hold_timer.start()

    def on_exit_release(self):
        self._cancel_exit_hold(reset_text=True)

    def _cancel_exit_hold(self, reset_text: bool = False):
        if self._exit_hold_timer.isActive():
            self._exit_hold_timer.stop()
        if reset_text and self.btn_kill_app is not None:
            self.btn_kill_app.setText(getattr(self, "_kill_btn_default_text", "EXIT APPLICATION"))

    def _exit_application_now(self):
        if not self.manual_mode:
            self._cancel_exit_hold(reset_text=True)
            return
        self._send_karel("APP_EXIT")
        self._log("Exit Application triggered (2s hold)")
        QtWidgets.QApplication.quit()

    def _send_karel(self, command: str, **fields):
        if self.karel_if is None:
            return
        self.karel_if.send(command, **fields)

    def _poll_karel_status(self):
        if self.karel_if is None:
            return

        status = self.karel_if.poll_status()
        if not status:
            return

        changed = False
        sync_remote_state = status.get("sync", "0").strip().upper() in {"1", "ON", "TRUE", "YES"}

        if sync_remote_state:
            state_name = status.get("state")
            if state_name in RobotState.__members__:
                remote_state = RobotState[state_name]
                if remote_state != self.state:
                    self.state = remote_state
                    changed = True

            manual_value = status.get("manual", "")
            if manual_value:
                normalized = manual_value.strip().upper()
                remote_manual = normalized in {"1", "ON", "TRUE", "YES"}
                if remote_manual != self.manual_mode:
                    self.manual_mode = remote_manual
                    changed = True

        message = status.get("message", "")
        if message:
            self._log(f"KAREL: {message}")

        if changed:
            self._update_status()
            self._update_ui_enabled()

    # ----------------------
    # UI helpers
    # ----------------------
    def _update_status(self, lite: bool = False):
        msg = f"State: {self.state.name} | Speed: {self.speed_override}% | Manual: {'ON' if self.manual_mode else 'OFF'}"
        if self.lbl_status is not None:
            self.lbl_status.setText(msg)
        if self.lbl_jog_status is not None:
            self.lbl_jog_status.setText(msg)
        if self.lbl_maint_status is not None:
            self.lbl_maint_status.setText(msg)
        if not lite:
            self._log(msg)

    def _update_ui_enabled(self):
        running = self.state == RobotState.AUTO_RUNNING
        fault = self.state == RobotState.AUTO_FAULTED
        in_jogging = self.manual_mode and self.state == RobotState.MANUAL_JOGGING
        in_handguide = self.manual_mode and self.state == RobotState.HAND_GUIDE
        app_controls_enabled = not self.manual_mode

        if not in_jogging:
            self._stop_all_jogs("leaving JOGGING")

        if self.btn_start is not None:
            self.btn_start.setEnabled(app_controls_enabled and (not running) and (not fault))
        if self.btn_stop is not None:
            self.btn_stop.setEnabled(app_controls_enabled and running)
        if self.btn_abort is not None:
            self.btn_abort.setEnabled(app_controls_enabled)
        if self.btn_reset is not None:
            self.btn_reset.setEnabled(app_controls_enabled)

        if self.btn_toggle_manual is not None:
            self.btn_toggle_manual.setText("Manual Mode: ON" if self.manual_mode else "Manual Mode: OFF")

        if self.btn_mode_teach is not None:
            self.btn_mode_teach.setEnabled(self.manual_mode and (not fault))
            self.btn_mode_teach.setText("JOG (ACTIVE)" if in_jogging else "JOG")
        if self.btn_mode_handguide is not None:
            self.btn_mode_handguide.setEnabled(self.manual_mode and (not fault))
            self.btn_mode_handguide.setText("HAND GUIDE (ACTIVE)" if in_handguide else "HAND GUIDE")

        if self.btn_move_home is not None:
            self.btn_move_home.setEnabled(self.manual_mode and (not fault))
        if self.btn_move_shipping is not None:
            self.btn_move_shipping.setEnabled(self.manual_mode and (not fault))
        if self.btn_manual_reset is not None:
            self.btn_manual_reset.setEnabled(self.manual_mode)

        if self.rdo_jog_cartesian is not None:
            self.rdo_jog_cartesian.setEnabled(not fault)
        if self.rdo_jog_joint is not None:
            self.rdo_jog_joint.setEnabled(not fault)
        if self.rdo_jog_simple is not None:
            self.rdo_jog_simple.setEnabled(not fault)

        jog_buttons = [
            self.btn_jog_x_minus,
            self.btn_jog_x_plus,
            self.btn_jog_y_minus,
            self.btn_jog_y_plus,
            self.btn_jog_z_minus,
            self.btn_jog_z_plus,
            self.btn_jog_w_minus,
            self.btn_jog_w_plus,
            self.btn_jog_p_minus,
            self.btn_jog_p_plus,
            self.btn_jog_r_minus,
            self.btn_jog_r_plus,
        ]
        for button in jog_buttons:
            if button is not None:
                button.setEnabled(in_jogging and (not fault) and (self.jog_mode != "simple"))

        simple_buttons = [
            self.btn_simple_left,
            self.btn_simple_right,
            self.btn_simple_front,
            self.btn_simple_back,
            self.btn_simple_up,
            self.btn_simple_down,
            self.btn_simple_home,
        ]
        simple_mode = self.jog_mode == "simple"
        for button in simple_buttons:
            if button is not None:
                button.setEnabled(in_jogging and (not fault) and simple_mode)

        # Exit button: hidden unless Manual Mode is ON
        if self.btn_kill_app is not None:
            self.btn_kill_app.setVisible(self.manual_mode)
            self.btn_kill_app.setEnabled(self.manual_mode)
            if not self.manual_mode:
                self._cancel_exit_hold(reset_text=True)

    def _log(self, text: str):
        timestamp = QtCore.QDateTime.currentDateTime().toString("HH:mm:ss")
        line = f"[{timestamp}] {text}"
        if self.txt_log is not None:
            self.txt_log.append(line)
        if self.txt_maint_log is not None:
            self.txt_maint_log.append(line)

    def _toast(self, title: str, message: str):
        QtWidgets.QMessageBox.information(self, title, message)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = RobotHMI()
    window.showFullScreen()
    sys.exit(app.exec_())
