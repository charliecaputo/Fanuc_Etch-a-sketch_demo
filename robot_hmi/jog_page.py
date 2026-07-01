#!/usr/bin/env python3

# ─────────────────────────────────────────────
# jog_page.py
# Manual Cartesian jog screen
# ─────────────────────────────────────────────
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QPushButton, QLabel, QSlider,
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont
from std_msgs.msg import Float32


class JogPage(QWidget):
    """
    Full-screen jog page.
    Calls ros.send_jog(direction) on press, ros.send_jog('stop') on release.
    Calls ros.send_speed(value) when the slider changes.
    back_callback — called when the Back button is pressed.
    """

    def __init__(self, ros, back_callback):
        super().__init__()
        self.ros = ros
        self.back_callback = back_callback
        
        self._initializing_slider = True
        self.setStyleSheet("background-color: #111111;")
        self._build_ui()

    # =========================================================
    # UI Construction
    # =========================================================

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(24, 24, 24, 24)
        root.setSpacing(16)

        root.addLayout(self._build_header())
        root.addLayout(self._build_jog_grid())
        root.addWidget(self._build_speed_slider())

    def _build_header(self):
        row = QHBoxLayout()

        back_btn = QPushButton("← Back")
        back_btn.setFixedHeight(44)
        back_btn.setFixedWidth(110)
        back_btn.setFont(QFont("Arial", 11))
        back_btn.setStyleSheet("""
            QPushButton {
                background-color: #1a1a2a;
                color: #8888aa;
                border: 1px solid #2a2a4a;
                border-radius: 8px;
            }
            QPushButton:pressed {
                background-color: #252535;
            }
        """)
        back_btn.clicked.connect(self._on_back)

        title = QLabel("Manual Jog (Cartesian)")
        title.setFont(QFont("Arial", 16, QFont.Weight.Medium))
        title.setStyleSheet("color: #cccccc;")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)

        # NEW: switch button
        switch_btn = QPushButton("Joint →")
        switch_btn.setFixedHeight(44)
        switch_btn.setFixedWidth(110)
        switch_btn.setFont(QFont("Arial", 11))
        switch_btn.setStyleSheet("""
            QPushButton {
                background-color: #1a1a2a;
                color: #7ec8f0;
                border: 1px solid #2a2a4a;
                border-radius: 8px;
            }
            QPushButton:pressed {
                background-color: #252535;
            }
        """)
        switch_btn.clicked.connect(self._on_switch_mode)

        row.addWidget(back_btn)
        row.addStretch()
        row.addWidget(title)
        row.addStretch()
        row.addWidget(switch_btn)

        return row
    def _build_jog_grid(self):

        outer = QHBoxLayout()
        outer.setSpacing(50)

        # ----------------------------
        # Translation
        # ----------------------------
        trans = QVBoxLayout()

        title = QLabel("Translation")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("color:#cccccc;")
        title.setFont(QFont("Arial", 13, QFont.Weight.Bold))
        trans.addWidget(title)

        trans_grid = QGridLayout()
        trans_grid.setSpacing(10)

        trans_grid.addWidget(self._jog_btn("+X", "+x", "#1a3a5a", "#4a9fd5"), 0, 0)
        trans_grid.addWidget(self._jog_btn("-X", "-x", "#1a3a5a", "#4a9fd5"), 1, 0)

        trans_grid.addWidget(self._jog_btn("+Y", "+y", "#1a3a5a", "#4a9fd5"), 0, 1)
        trans_grid.addWidget(self._jog_btn("-Y", "-y", "#1a3a5a", "#4a9fd5"), 1, 1)

        trans_grid.addWidget(self._jog_btn("+Z", "+z", "#1a3a5a", "#4a9fd5"), 0, 2)
        trans_grid.addWidget(self._jog_btn("-Z", "-z", "#1a3a5a", "#4a9fd5"), 1, 2)

        trans.addLayout(trans_grid)

        # ----------------------------
        # Rotation
        # ----------------------------
        rot = QVBoxLayout()

        title = QLabel("Rotation")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("color:#cccccc;")
        title.setFont(QFont("Arial", 13, QFont.Weight.Bold))
        rot.addWidget(title)

        rot_grid = QGridLayout()
        rot_grid.setSpacing(10)

        rot_grid.addWidget(self._jog_btn("+Roll", "+roll", "#3a1a1a", "#d57a4a"), 0, 0)
        rot_grid.addWidget(self._jog_btn("-Roll", "-roll", "#3a1a1a", "#d57a4a"), 1, 0)

        rot_grid.addWidget(self._jog_btn("+Pitch", "+pitch", "#3a1a1a", "#d57a4a"), 0, 1)
        rot_grid.addWidget(self._jog_btn("-Pitch", "-pitch", "#3a1a1a", "#d57a4a"), 1, 1)

        rot_grid.addWidget(self._jog_btn("+Yaw", "+yaw", "#3a1a1a", "#d57a4a"), 0, 2)
        rot_grid.addWidget(self._jog_btn("-Yaw", "-yaw", "#3a1a1a", "#d57a4a"), 1, 2)

        rot.addLayout(rot_grid)

        outer.addLayout(trans)
        outer.addLayout(rot)

        return outer

    def _jog_btn(self, label: str, direction: str, bg: str, border: str):
        """Create a single jog button — sends direction on press, stop on release."""
        btn = QPushButton(label)
        btn.setFont(QFont("Arial", 16, QFont.Weight.Medium))
        btn.setMinimumHeight(90)
        btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {bg};
                color: #7ec8f0;
                border: 2px solid {border};
                border-radius: 12px;
            }}
            QPushButton:pressed {{
                background-color: #254a6a;
            }}
        """)
        btn.pressed.connect(lambda d=direction: self.ros.send_jog(d))
        btn.released.connect(lambda: self.ros.send_jog('stop'))
        return btn

    def _build_speed_slider(self):
        container = QWidget()
        container.setStyleSheet(
            "background: #1a1a2a; border: 1px solid #2a2a4a; border-radius: 5px;"
        )

        layout = QVBoxLayout(container)
        layout.setContentsMargins(14, 8, 14, 8)
        layout.setSpacing(8)

        speed = getattr(self.ros, "speed", 0.5)
        speed_percent = int(speed * 100)

        self.speed_label = QLabel(f"Speed: {speed_percent}%")
        self.speed_label.setFont(QFont("Arial", 16, QFont.Weight.Medium))
        self.speed_label.setStyleSheet("color: #7ec8f0;")
        self.speed_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.speed_label.setFixedHeight(60)

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setFixedHeight(60)
        self.slider.setMinimum(0)
        self.slider.setMaximum(100)
        self.slider.setValue(speed_percent)
        self._initializing_slider = False
        self.slider.setTickInterval(10)
        self.slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.slider.setStyleSheet("""
            QSlider::groove:horizontal {
                height: 16px;
                background: #2a2a4a;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #4a9fd5;
                border: 2px solid #7ec8f0;
                width: 12px;
                height: 32px;
                margin: -10px;
                border-radius: 14px;
            }
            QSlider::sub-page:horizontal {
                background: #1a3a5a;
                border-radius: 4px;
            }
        """)
        self.slider.valueChanged.connect(self._on_speed_changed)        

        layout.addWidget(self.speed_label)
        layout.addWidget(self.slider)

        return container

    # =========================================================
    # Page visibility — sync speed whenever this page is shown
    # =========================================================

    def showEvent(self, event):
        super().showEvent(event)
        self._refresh_speed()

    def _refresh_speed(self):
        """Read speed from the ROS node and update the slider + label
        without publishing a new speed command."""
        speed_percent = int(getattr(self.ros, "speed", 0.5) * 100)
        self.slider.blockSignals(True)
        self.slider.setValue(speed_percent)
        self.slider.blockSignals(False)
        self.speed_label.setText(f"Speed: {speed_percent}%")

    # =========================================================
    # Callbacks
    # =========================================================

    def _on_back(self):
        # Send stop before leaving in case a button is still held
        self.ros.send_jog('stop')
        self.back_callback()

    def _on_speed_changed(self, value: int):
        if getattr(self, "_initializing_slider", False):
            return
        self.speed_label.setText(f"Speed: {value}%")
        self.ros.send_speed(value / 100.0)
        
    def _on_switch_mode(self):
        self.ros.send_jog('stop')
        self.back_callback("joint")
