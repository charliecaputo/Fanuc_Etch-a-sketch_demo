#!/usr/bin/env python3

# ─────────────────────────────────────────────
# widgets.py
# Reusable UI widgets
# ─────────────────────────────────────────────

from PyQt6.QtWidgets import QPushButton, QVBoxLayout, QLabel
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont


class ModeButton(QPushButton):

    def __init__(self, label, sublabel, mode_key):
        super().__init__()
        self.mode_key = mode_key
        self.is_active = False

        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.setSpacing(4)

        self.label_widget = QLabel(label)
        self.label_widget.setFont(QFont("Arial", 14, QFont.Weight.Medium))
        self.label_widget.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.sub_widget = QLabel(sublabel)
        self.sub_widget.setFont(QFont("Arial", 10))
        self.sub_widget.setAlignment(Qt.AlignmentFlag.AlignCenter)

        layout.addWidget(self.label_widget)
        layout.addWidget(self.sub_widget)

        self.setMinimumHeight(90)
        self._set_inactive()

    def _set_active(self):
        self.is_active = True
        self.setStyleSheet("""
            QPushButton {
                background-color: #00A651;
                border: 2px solid #00A651;
                border-radius: 12px;
            }
        """)
        self.label_widget.setStyleSheet("""
            color: #ffffff;
            background: 00A651;
        """)
        self.sub_widget.setStyleSheet("""
            color: #ffffff;
            background: 00A651;
        """)

    def _set_inactive(self):
        self.is_active = False
        self.setStyleSheet("""
            QPushButton {
                background-color: #004d25;
                border: 1px solid #004d25;
                border-radius: 12px;
            }
            QPushButton:pressed {
                background-color: #03ff7c;
            }
        """)
        self.label_widget.setStyleSheet("""
            color: #ffffff;
            background: 004d25;
        """)
        self.sub_widget.setStyleSheet("""
            color: #ffffff;
            background: 004d25;
        """)

    def set_active(self, active: bool):
        if active:
            self._set_active()
        else:
            self._set_inactive()
