#!/usr/bin/env python3

# ─────────────────────────────────────────────
# estop_controller.py
# E-STOP hold-to-reset state machine. Knows
# nothing about ROS or subprocesses directly —
# it drives the estop button and calls back into
# whatever callbacks it's given.
# ─────────────────────────────────────────────

from PyQt6.QtWidgets import QPushButton
from PyQt6.QtCore import QTimer

import styles


class EstopController:
    """Owns the E-STOP button's visual state and the "hold 3s to
    reset" timing logic.

    Callbacks (all no-arg, except set_mode_fn):
        log_fn        -- str -> None, prints to the terminal widget
        trigger_fn    -- called when E-STOP fires: should stop any
                          in-flight motion and switch to the main page
        kill_fn       -- called when E-STOP fires: should kill any
                          running processes (manual init, encoder teleop, etc)
        reset_ui_fn   -- called when E-STOP fires: should reset mode
                          buttons back to their idle state
        set_mode_fn   -- str -> None, called with the current logical
                          mode ("ESTOPPED", "ESTOP RELEASED", ...)
    """

    def __init__(
        self,
        estop_btn: QPushButton,
        log_fn,
        trigger_fn,
        kill_fn,
        reset_ui_fn,
        set_mode_fn,
        hold_required_ms: int = 3000,
    ):
        self.estop_btn = estop_btn
        self.log = log_fn
        self.trigger_fn = trigger_fn
        self.kill_fn = kill_fn
        self.reset_ui_fn = reset_ui_fn
        self.set_mode_fn = set_mode_fn

        self.latched = False
        self.reset_ready = False
        self.hold_time = 0
        self.hold_required = hold_required_ms

        self.hold_timer = QTimer()
        self.hold_timer.setInterval(50)
        self.hold_timer.timeout.connect(self._update_hold)

        self.setup_button()

    # =========================================================
    # Trigger / release
    # =========================================================

    def trigger(self):
        self.log("E-STOP ACTIVATED")

        self.trigger_fn()
        self.kill_fn()
        self.reset_ui_fn()

        self.set_mode_fn("ESTOPPED")

        self.estop_btn.setText("⛔ HOLD 3s TO RESET")
        self.estop_btn.setStyleSheet(styles.ESTOP_LATCHED)

        self.latched = True
        self.reset_ready = True

    def release(self):
        self.log("E-STOP RELEASED")
        self.set_mode_fn("ESTOP RELEASED")

        self.latched = False
        self.reset_ready = False
        self.hold_timer.stop()
        self.hold_time = 0

        self.setup_button()
        self.log("System fully restored and reusable, please re-enable the robot")

    # =========================================================
    # Hold-to-reset timing
    # =========================================================

    def start_hold(self):
        if not self.latched or not self.reset_ready:
            return
        self.hold_time = 0
        self.hold_timer.stop()
        self.hold_timer.start()

    def stop_hold(self):
        if not self.latched:
            return
        self.hold_timer.stop()
        self.hold_time = 0
        if self.latched:
            self.estop_btn.setText("⛔ HOLD 3s TO RESET")

    def _update_hold(self):
        if not self.latched or not self.reset_ready:
            self.hold_timer.stop()
            return

        self.hold_time += 50
        progress = min(self.hold_time / self.hold_required, 1.0)

        bar = int(progress * 20)
        self.estop_btn.setText(
            "⛔ RESETTING [" + "*" * bar + "-" * (20 - bar) + "]"
        )
        self.estop_btn.setStyleSheet(styles.estop_hold_progress(progress))

        if self.hold_time >= self.hold_required:
            self.release()

    # =========================================================
    # Button wiring
    # =========================================================

    def setup_button(self):
        """Always restores E-STOP to its default functional state."""
        for signal_name in ("clicked", "pressed", "released"):
            try:
                getattr(self.estop_btn, signal_name).disconnect()
            except Exception:
                pass

        self.estop_btn.clicked.connect(self.trigger)
        self.estop_btn.pressed.connect(self.start_hold)
        self.estop_btn.released.connect(self.stop_hold)

        self.estop_btn.setStyleSheet(styles.ESTOP_DEFAULT)
        self.estop_btn.setText("⛔ E-STOP")
        self.estop_btn.setDown(False)
        self.estop_btn.setChecked(False)
        self.estop_btn.blockSignals(False)
