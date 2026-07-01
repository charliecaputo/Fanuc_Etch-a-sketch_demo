#!/usr/bin/env python3

# ─────────────────────────────────────────────
# styles.py
# QSS stylesheet strings for the HMI window.
# Pulled out of window.py so layout code isn't
# buried under CSS-in-strings.
# ─────────────────────────────────────────────

ROOT_BACKGROUND = "background-color: #000000;"

HEADER_LABEL = "color: #FFD100;"

MIN_BUTTON = """
    QPushButton {
        background-color: #1a1a1a;
        color: #FFFFFF;
        border: 1px solid #FFFFFF;
        border-radius: 8px;
        font-size: 18px;
    }
    QPushButton:pressed {
        background-color: #333333;
    }
"""

CLOSE_BUTTON = """
    QPushButton {
        background-color: #330000;
        color: #D91433;
        border: 1px solid #D91433;
        border-radius: 8px;
        font-size: 18px;
        font-weight: bold;
    }
    QPushButton:pressed {
        background-color: #550000;
    }
"""

STATUS_BAR = (
    "background: #00a651; border: 1px solid #ffffff; border-radius: 10px;"
)

TERM_OUTPUT = """
    QPlainTextEdit {
        background-color: #000000;
        color: #FFD100;
        border: 1px solid #00a651;
        border-radius: 8px;
    }
"""

ESTOP_DEFAULT = """
    QPushButton {
        background-color: #2a0000;
        color: #fc0808;
        border: 2px solid #cc0000;
        border-radius: 12px;
    }
    QPushButton:pressed {
        background-color: #440000;
    }
"""

ESTOP_LATCHED = """
    QPushButton {
        background-color: #cc0000;
        color: white;
        border: 3px solid #ff0000;
        border-radius: 12px;
        font-size: 18px;
    }
"""


def estop_hold_progress(progress: float) -> str:
    """Style for the E-STOP button while it's being held down to reset.
    `progress` is 0.0-1.0.
    """
    return f"""
        QPushButton {{
            background-color: rgba(200, 0, 0, {0.5 + 0.5 * progress});
            color: white;
            border: 3px solid #ff0000;
            border-radius: 12px;
            font-size: 16px;
        }}
    """
