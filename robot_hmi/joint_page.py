#!/usr/bin/env python3

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QPushButton, QLabel, QSlider,
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont


class JointPage(QWidget):

    def __init__(self, ros, back_callback):
        super().__init__()
        self.ros = ros
        self.back_callback = back_callback
        self._initializing_slider = True   # prevent spurious publish during construction
        self.setStyleSheet("background-color: #111111;")
        self._build_ui()
        self._initializing_slider = False

    # =========================================================
    # UI
    # =========================================================

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(24, 24, 24, 24)
        root.setSpacing(16)

        root.addLayout(self._build_header())
        root.addLayout(self._build_joint_grid())
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

        title = QLabel("Manual Jog (Joint)")
        title.setFont(QFont("Arial", 16, QFont.Weight.Medium))
        title.setStyleSheet("color: #cccccc;")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)

        switch_btn = QPushButton("Cartesian →")
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

    def _build_joint_grid(self):

        outer = QVBoxLayout()
        outer.setSpacing(50)

        title = QLabel("Joints")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("color:#cccccc;")
        title.setFont(QFont("Arial", 13, QFont.Weight.Bold))
        outer.addWidget(title)

        grid = QGridLayout()
        grid.setSpacing(10)

        joints = [
            ("+J1", "+j1"), ("+J2", "+j2"),
            ("+J3", "+j3"), ("+J4", "+j4"),
            ("+J5", "+j5"), ("+J6", "+j6"),
            ("-J1", "-j1"), ("-J2", "-j2"),
            ("-J3", "-j3"), ("-J4", "-j4"),
            ("-J5", "-j5"), ("-J6", "-j6"),
        ]

        # 2 rows × 6 columns
        for i, (label, cmd) in enumerate(joints):
            row = i // 6
            col = i % 6

            grid.addWidget(
                self._jog_btn(label, cmd, "#2a1a3a", "#a04ad5"),
                row,
                col
            )

        outer.addLayout(grid)
        return outer
    def _jog_btn(self, label, direction, bg, border):
        btn = QPushButton(label)
        btn.setFont(QFont("Arial", 16, QFont.Weight.Medium))
        btn.setMinimumHeight(90)
        btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {bg};
                color: #d0a7ff;
                border: 2px solid {border};
                border-radius: 12px;
            }}
            QPushButton:pressed {{
                background-color: #3a2a5a;
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
        speed = getattr(self.ros, "speed", 0.5)   # default = 0.5
        speed_percent = int(speed * 100)
        
        self.speed_label = QLabel(f"Speed: {speed_percent}%")
        self.speed_label.setStyleSheet("color:#7ec8f0;")
        self.speed_label.setFont(QFont("Arial", 16, QFont.Weight.Medium))
        self.speed_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.speed_label.setFixedHeight(60)

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setFixedHeight(60)
        self.slider.setMinimum(0)
        self.slider.setMaximum(100)
        self.slider.setValue(speed_percent)
        self.slider.valueChanged.connect(self._on_speed)
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
        self.ros.send_jog('stop')
        self.back_callback("cartesian")

    def _on_switch_mode(self):
        self.ros.send_jog('stop')
        self.back_callback("cartesian")

    def _on_speed(self, v):
        if getattr(self, "_initializing_slider", False):
            return
        self.speed_label.setText(f"Speed: {v}%")
        self.ros.send_speed(v / 100.0)
        #self.ros.send_speed(self.ros.speed) 
