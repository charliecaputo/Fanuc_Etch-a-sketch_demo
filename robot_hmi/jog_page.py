#!/usr/bin/env python3

# ─────────────────────────────────────────────
# jog_page.py
#
# Manual Cartesian jogging UI for a robot arm.
#
# This page provides:
#   • 6-DOF Cartesian jog controls (X/Y/Z + Roll/Pitch/Yaw)
#   • Continuous motion while buttons are held
#   • Immediate stop on button release
#   • A speed control slider (0–100%)
#   • Navigation back to the main UI or joint mode
#
# The page communicates with a ROS-like interface through:
#   ros.send_jog(direction)
#   ros.send_speed(value)
# ─────────────────────────────────────────────

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QPushButton, QLabel, QSlider,
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont
from std_msgs.msg import Float32  # (unused here, likely legacy or future use)
import styles


class JogPage(QWidget):
    """
    Full-screen manual jogging interface.

    Behavior:
        • Press and hold jog buttons → continuous motion
        • Release → stop motion
        • Slider → sets global velocity scaling
        • Back button → returns to previous screen

    back_callback:
        Function called when user exits this page.
        May optionally accept a mode argument (e.g. "joint").
    """

    def __init__(self, ros, back_callback):
        super().__init__()

        # ROS/robot interface abstraction
        self.ros = ros

        # Navigation callback (switch pages / modes)
        self.back_callback = back_callback

        # Flag used to prevent slider event spam during initialization
        self._initializing_slider = True

        # Set dark background theme for full-screen control UI
        self.setStyleSheet("background-color: #000000;")

        # Build UI layout
        self._build_ui()

    # =========================================================
    # UI Construction
    # =========================================================

    def _build_ui(self):
        """Top-level layout assembly for the page."""
        root = QVBoxLayout(self)
        root.setContentsMargins(24, 24, 24, 24)
        root.setSpacing(16)

        # Header, jog controls, and speed slider
        root.addLayout(self._build_header())
        root.addLayout(self._build_jog_grid())
        root.addWidget(self._build_speed_slider())

    def _build_header(self):
        """Creates top bar with navigation and window controls."""
        row = QHBoxLayout()

        # Back navigation button
        back_btn = QPushButton("← Back")
        back_btn.setFixedHeight(44)
        back_btn.setFixedWidth(110)
        back_btn.setFont(QFont("Arial", 14))
        back_btn.setStyleSheet("""
            QPushButton {
                background-color: #696868;
                color: #ffffff;
                border: 1px solid #a8a8a8;
                border-radius: 8px;
            }
            QPushButton:pressed {
                background-color: #a8a8a8;
            }
        """)
        back_btn.clicked.connect(self._on_back)

        # Page title
        title = QLabel("Manual Jog (Cartesian)")
        title.setFont(QFont("Arial", 20, QFont.Weight.Medium))
        title.setStyleSheet("color: #ffffff;")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)

        # Mode switch button (Cartesian → Joint)
        switch_btn = QPushButton("Joint →")
        switch_btn.setFixedHeight(44)
        switch_btn.setFixedWidth(110)
        switch_btn.setFont(QFont("Arial", 14))
        switch_btn.setStyleSheet("""
            QPushButton {
                background-color: #696969;
                color: #ffffff;
                border: 1px solid #a8a8a8;
                border-radius: 8px;
            }
            QPushButton:pressed {
                background-color: #a8a8a8;
            }
        """)
        switch_btn.clicked.connect(self._on_switch_mode)

        # Window minimize button
        min_btn = QPushButton("—")
        min_btn.setFixedSize(40, 40)
        min_btn.setStyleSheet(styles.MIN_BUTTON)
        min_btn.clicked.connect(lambda: self.window().showMinimized())

        # Window close button
        close_btn = QPushButton("✕")
        close_btn.setFixedSize(40, 40)
        close_btn.setStyleSheet(styles.CLOSE_BUTTON)
        close_btn.clicked.connect(lambda: self.window().close())

        # Layout arrangement
        row.addWidget(back_btn)
        row.addWidget(switch_btn)
        row.addStretch()
        row.addWidget(title)
        row.addStretch()
        row.addWidget(min_btn)
        row.addWidget(close_btn)

        return row

    def _build_jog_grid(self):
        """Builds the 6-DOF jog control grid (translation + rotation)."""

        outer = QHBoxLayout()
        outer.setSpacing(50)

        # ----------------------------
        # Translation controls (X/Y/Z)
        # ----------------------------
        trans = QVBoxLayout()

        title = QLabel("Translation")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("color:#ffffff;")
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        trans.addWidget(title)

        trans_grid = QGridLayout()
        trans_grid.setSpacing(10)

        # X axis
        trans_grid.addWidget(self._jog_btn("+X", "+x", "#004d25", "#ffffff"), 0, 0)
        trans_grid.addWidget(self._jog_btn("-X", "-x", "#004d25", "#ffffff"), 1, 0)

        # Y axis
        trans_grid.addWidget(self._jog_btn("+Y", "+y", "#004d25", "#ffffff"), 0, 1)
        trans_grid.addWidget(self._jog_btn("-Y", "-y", "#004d25", "#ffffff"), 1, 1)

        # Z axis
        trans_grid.addWidget(self._jog_btn("+Z", "+z", "#004d25", "#ffffff"), 0, 2)
        trans_grid.addWidget(self._jog_btn("-Z", "-z", "#004d25", "#ffffff"), 1, 2)

        trans.addLayout(trans_grid)

        # ----------------------------
        # Rotation controls (Roll/Pitch/Yaw)
        # ----------------------------
        rot = QVBoxLayout()

        title = QLabel("Rotation")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("color:#ffffff;")
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        rot.addWidget(title)

        rot_grid = QGridLayout()
        rot_grid.setSpacing(10)

        # Roll
        rot_grid.addWidget(self._jog_btn("+Roll", "+roll", "#004d25", "#ffffff"), 0, 0)
        rot_grid.addWidget(self._jog_btn("-Roll", "-roll", "#004d25", "#ffffff"), 1, 0)

        # Pitch
        rot_grid.addWidget(self._jog_btn("+Pitch", "+pitch", "#004d25", "#ffffff"), 0, 1)
        rot_grid.addWidget(self._jog_btn("-Pitch", "-pitch", "#004d25", "#ffffff"), 1, 1)

        # Yaw
        rot_grid.addWidget(self._jog_btn("+Yaw", "+yaw", "#004d25", "#ffffff"), 0, 2)
        rot_grid.addWidget(self._jog_btn("-Yaw", "-yaw", "#004d25", "#ffffff"), 1, 2)

        rot.addLayout(rot_grid)

        outer.addLayout(trans)
        outer.addLayout(rot)

        return outer

    def _jog_btn(self, label: str, direction: str, bg: str, border: str):
        """
        Factory for jog buttons.

        Each button:
            • Sends continuous jog command on press
            • Sends 'stop' on release
        """
        btn = QPushButton(label)
        btn.setFont(QFont("Arial", 16, QFont.Weight.Medium))
        btn.setMinimumHeight(90)

        btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {bg};
                color: #ffffff;
                border: 2px solid {border};
                border-radius: 12px;
            }}
            QPushButton:pressed {{
                background-color: #00A651;
            }}
        """)

        # Continuous motion while held
        btn.pressed.connect(lambda d=direction: self.ros.send_jog(d))

        # Stop motion immediately on release
        btn.released.connect(lambda: self.ros.send_jog('stop'))

        return btn

    def _build_speed_slider(self):
        """Creates speed scaling slider (0–100%)."""

        container = QWidget()
        container.setStyleSheet(
            "background: #9c0e24; border: 2px solid #FFD100; border-radius: 5px;"
        )

        layout = QVBoxLayout(container)
        layout.setContentsMargins(14, 8, 14, 8)
        layout.setSpacing(8)

        # Read current speed from ROS interface (default 0.5 if missing)
        speed = getattr(self.ros, "speed", 0.5)
        speed_percent = int(speed * 100)

        # Label showing current speed
        self.speed_label = QLabel(f"Speed: {speed_percent}%")
        self.speed_label.setFont(QFont("Arial", 16, QFont.Weight.Medium))
        self.speed_label.setStyleSheet("color: #FFD100;")
        self.speed_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.speed_label.setFixedHeight(60)

        # Slider setup
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setFixedHeight(100)
        self.slider.setMinimum(0)
        self.slider.setMaximum(100)
        self.slider.setValue(speed_percent)

        # Allow first sync without triggering ROS command
        self._initializing_slider = False

        self.slider.setTickInterval(10)
        self.slider.setTickPosition(QSlider.TickPosition.TicksBelow)

        self.slider.setStyleSheet("""
            QSlider::groove:horizontal {
                height: 24px;
                background: #876f00;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #ffffff;
                border: 2px solid #a8a8a8;
                width: 48px;
                height: 48px;
                margin: -10px;
                border-radius: 14px;
            }
            QSlider::sub-page:horizontal {
                background: #FFD100;
                border-radius: 4px;
            }
        """)

        # Update speed when user drags slider
        self.slider.valueChanged.connect(self._on_speed_changed)

        layout.addWidget(self.speed_label)
        layout.addWidget(self.slider)

        return container

    # =========================================================
    # Lifecycle: keep UI in sync when page is shown
    # =========================================================

    def showEvent(self, event):
        """Triggered when page becomes visible."""
        super().showEvent(event)
        self._refresh_speed()

    def _refresh_speed(self):
        """Sync slider with ROS speed without sending command."""
        speed_percent = int(getattr(self.ros, "speed", 0.5) * 100)

        self.slider.blockSignals(True)
        self.slider.setValue(speed_percent)
        self.slider.blockSignals(False)

        self.speed_label.setText(f"Speed: {speed_percent}%")

    # =========================================================
    # Callbacks
    # =========================================================

    def _on_back(self):
        """Exit jog page safely (stop motion first)."""
        self.ros.send_jog('stop')
        self.back_callback()

    def _on_speed_changed(self, value: int):
        """Send updated speed scaling to robot."""
        if getattr(self, "_initializing_slider", False):
            return

        self.speed_label.setText(f"Speed: {value}%")
        self.ros.send_speed(value / 100.0)

    def _on_switch_mode(self):
        """Switch from Cartesian jog page to joint jog page."""
        self.ros.send_jog('stop')
        self.back_callback("joint")
