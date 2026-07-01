#!/usr/bin/env python3

# ─────────────────────────────────────────────
# main.py
# Entry point — run this file to launch the HMI
# ─────────────────────────────────────────────

import sys
from PyQt6.QtWidgets import QApplication

from ros_bridge import ROSBridge
from window import HMIWindow


def main():
    app = QApplication(sys.argv)
    ros = ROSBridge()
    window = HMIWindow(ros)
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
