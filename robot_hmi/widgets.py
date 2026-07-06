#!/usr/bin/env python3

# ─────────────────────────────────────────────
# widgets.py
#
# Reusable UI components for the HMI.
#
# Currently includes:
#   • ModeButton — a styled toggle-like button used
#     for selecting robot operating modes
# ─────────────────────────────────────────────

from PyQt6.QtWidgets import QPushButton, QVBoxLayout, QLabel
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont


class ModeButton(QPushButton):
    """
    A composite QPushButton used for mode selection.

    Structure:
        Label (main mode name)
        Sub-label (description or status)
    
    The button supports two visual states:
        • Active   → highlighted (green)
        • Inactive → default dark theme

    Intended usage:
        Mode selection panels (e.g., Jog, Ship program, Program start, Home program, etc.)
    """

    def __init__(self, label, sublabel, mode_key):
        super().__init__()

        # Identifier used by application logic
        self.mode_key = mode_key

        # Tracks current visual state
        self.is_active = False

        # Build internal vertical layout inside the button
        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.setSpacing(4)

        # Main label (mode name)
        self.label_widget = QLabel(label)
        self.label_widget.setFont(QFont("Arial", 14, QFont.Weight.Medium))
        self.label_widget.setAlignment(Qt.AlignmentFlag.AlignCenter)

        # Secondary label (description / hint)
        self.sub_widget = QLabel(sublabel)
        self.sub_widget.setFont(QFont("Arial", 10))
        self.sub_widget.setAlignment(Qt.AlignmentFlag.AlignCenter)

        # Add widgets to button layout
        layout.addWidget(self.label_widget)
        layout.addWidget(self.sub_widget)

        # Ensure consistent sizing
        self.setMinimumHeight(90)

        # Start in inactive visual state
        self._set_inactive()

    # =========================================================
    # Visual state management
    # =========================================================

    def _set_active(self):
        """Apply active (selected) styling."""

        self.is_active = True

        # Main button styling (green highlight)
        self.setStyleSheet("""
            QPushButton {
                background-color: #00A651;
                border: 2px solid #00A651;
                border-radius: 12px;
            }
            QPushButton:disabled {
                background-color: #666666;
                border: 1px solid #444444;
                color: #bbbbbb;
            }
        """)

        # Label styling in active state
        self.label_widget.setStyleSheet("""
            color: #ffffff;
            background: #00A651;
        """)

        # Sub-label styling in active state
        self.sub_widget.setStyleSheet("""
            color: #ffffff;
            background: #00A651;
        """)

    def _set_inactive(self):
        """Apply inactive (default) styling."""

        self.is_active = False

        # Default dark green styling
        self.setStyleSheet("""
            QPushButton {
                background-color: #004d25;
                border: 1px solid #004d25;
                border-radius: 12px;
            }
            QPushButton:pressed {
                background-color: #03ff7c;
            }
            QPushButton:disabled {
                background-color: #444444;
                border: 1px solid #555555;
                color: #bbbbbb;
            }
        """)

        # Label styling in inactive state
        self.label_widget.setStyleSheet("""
            QLabel {
                color: #ffffff;
                background: #004d25;
            }

            QLabel:disabled {
                color: #999999;
                background: #444444;
            }
        """)

        # Sub-label styling in inactive state
        self.sub_widget.setStyleSheet("""
            QLabel {
                color: #ffffff;
                background: #004d25;
            }

            QLabel:disabled {
                color: #999999;
                background: #444444;
            }
        """)

    def set_active(self, active: bool):
        """
        Public API to toggle button state.

        Args:
            active (bool):
                True  → highlight as selected mode
                False → revert to default appearance
        """
        if active:
            self._set_active()
        else:
            self._set_inactive()
