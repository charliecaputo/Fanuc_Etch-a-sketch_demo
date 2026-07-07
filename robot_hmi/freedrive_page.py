#!/usr/bin/env python3

from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QLabel,
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont

import styles


class FreedrivePage(QWidget):
    """
    Manual Guided Motion (Free Drive)

    Hold the button to enable MGT.
    Release immediately disables MGT.
    """

    FLAG = 8

    def __init__(self, ros, back_callback):
        super().__init__()

        self.ros = ros
        self.back_callback = back_callback

        self.setStyleSheet("background-color: #000000;")
        
        self._freedrive_enabled = False
        self._lock_armed = False      # Lock button is checked
        self._lock_latched = False    # Freedrive is latched on

        self._build_ui()

    # ---------------------------------------------------------
    # UI
    # ---------------------------------------------------------

    def _build_ui(self):

        root = QVBoxLayout(self)
        root.setContentsMargins(24, 24, 24, 24)
        root.setSpacing(30)

        root.addLayout(self._build_header())

        root.addStretch()

        title = QLabel("Manual Guided Motion")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setFont(QFont("Arial", 24, QFont.Weight.Bold))
        title.setStyleSheet("color:white;")

        info = QLabel(
            "Press and hold the button below to enable Free Drive. \n THIS CODE IS UNTESTED ON THE REAL ROBOT, PUBLISHING TO FLAG 8. \n MAKE SURE TO ENABLE IN COLLAB SETTINGS."
        )
        info.setAlignment(Qt.AlignmentFlag.AlignCenter)
        info.setFont(QFont("Arial", 16))
        info.setStyleSheet("color:#CCCCCC;")

        root.addWidget(title)
        root.addWidget(info)

        root.addSpacing(40)

        self.enable_btn = QPushButton("HOLD FOR\nFREE DRIVE")
        self.enable_btn.setMinimumSize(420, 260)
        self.enable_btn.setFont(QFont("Arial", 28, QFont.Weight.Bold))

        self.enable_btn.setStyleSheet("""
            QPushButton {
                background-color: #005500;
                color: white;
                border: 4px solid #00ff66;
                border-radius: 20px;
            }

            QPushButton:pressed {
                background-color: #00A651;
            }
        """)

        self.enable_btn.pressed.connect(self._enable_freedrive)
        self.enable_btn.released.connect(self._disable_freedrive)

        button_row = QHBoxLayout()
        button_row.setSpacing(15)

        button_row.addStretch()
        button_row.addWidget(self.enable_btn)

        self.lock_btn = QPushButton("🔒")
        self.lock_btn.setCheckable(True)
        self.lock_btn.setFixedSize(80, 80)
        self.lock_btn.setFont(QFont("Arial", 22))
        self.lock_btn.setStyleSheet("""
        QPushButton {
            background:#444444;
            color:white;
            border:3px solid #888888;
            border-radius:12px;
        }

        QPushButton:checked {
            background:#0077cc;
            border:3px solid #33bbff;
        }

        QPushButton:pressed {
            background:#666666;
        }
        """)

        self.lock_btn.clicked.connect(self._lock_clicked)

        button_row.addWidget(self.lock_btn)
        button_row.addStretch()

        root.addLayout(button_row)

        root.addStretch()
        
    def _build_header(self):

        row = QHBoxLayout()

        back_btn = QPushButton("← Back")
        back_btn.setFixedSize(110, 44)
        back_btn.setFont(QFont("Arial", 14))
        back_btn.setStyleSheet("""
            QPushButton {
                background:#696868;
                color:white;
                border:1px solid #A8A8A8;
                border-radius:8px;
            }

            QPushButton:pressed {
                background:#A8A8A8;
            }
        """)
        back_btn.clicked.connect(self._on_back)

        title = QLabel("Free Drive")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setFont(QFont("Arial", 20))
        title.setStyleSheet("color:white;")

        min_btn = QPushButton("—")
        min_btn.setFixedSize(40, 40)
        min_btn.setStyleSheet(styles.MIN_BUTTON)
        min_btn.clicked.connect(lambda: self.window().showMinimized())

        close_btn = QPushButton("✕")
        close_btn.setFixedSize(40, 40)
        close_btn.setStyleSheet(styles.CLOSE_BUTTON)
        close_btn.clicked.connect(lambda: self.window().close())

        row.addWidget(back_btn)
        row.addStretch()
        row.addWidget(title)
        row.addStretch()
        row.addWidget(min_btn)
        row.addWidget(close_btn)

        return row

    # ---------------------------------------------------------
    # Free Drive
    # ---------------------------------------------------------

    def _enable_freedrive(self):
        # If already latched, pressing HOLD again unlocks everything.
        if self._lock_latched:
            self._lock_latched = False
            self.lock_btn.setChecked(False)
            self._lock_armed = False
            self._disable_freedrive()
            return

        if not self._freedrive_enabled:
            self._freedrive_enabled = True
            self.ros.set_flag(self.FLAG, True)

        # HOLD + LOCK -> latch on
        if self._lock_armed:
            self._lock_latched = True

    def _disable_freedrive(self):
        # Ignore release if we're latched
        if self._lock_latched:
            return

        if self._freedrive_enabled:
            self._freedrive_enabled = False
            self.ros.set_flag(self.FLAG, False)
            
    def _lock_clicked(self):
        if self._lock_latched:
            # Unlock while latched
            self._lock_latched = False
            self.lock_btn.setChecked(False)
            self._lock_armed = False
            self._disable_freedrive()
        else:
            self._lock_armed = self.lock_btn.isChecked()

    # ---------------------------------------------------------
    # Safety
    # ---------------------------------------------------------

    def hideEvent(self, event):
        self._lock_latched = False
        self._lock_armed = False

        if hasattr(self, "lock_btn"):
            self.lock_btn.setChecked(False)

        self._disable_freedrive()
        super().hideEvent(event)

    def closeEvent(self, event):
        self._lock_latched = False
        self._lock_armed = False

        if hasattr(self, "lock_btn"):
            self.lock_btn.setChecked(False)

        self._disable_freedrive()
        super().closeEvent(event)
        
        self._disabled = False

    def _on_back(self):
        self._lock_latched = False
        self._lock_armed = False

        if hasattr(self, "lock_btn"):
            self.lock_btn.setChecked(False)

        self._disable_freedrive()
        self.back_callback()
