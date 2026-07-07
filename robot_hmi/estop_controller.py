#!/usr/bin/env python3

# ─────────────────────────────────────────────
# estop_controller.py
#
# Handles the Emergency Stop (E-STOP) button logic.
#
# Responsibilities:
#   • Manage the E-STOP button appearance
#   • Latch the E-STOP once activated
#   • Require the user to hold the button for 3 seconds
#     before allowing the system to reset
#   • Notify the rest of the application through callbacks
#
# This class intentionally has no knowledge of ROS,
# subprocesses, or robot-specific code. Instead, it calls
# user-provided callback functions whenever an action is
# required.
# ─────────────────────────────────────────────

from PyQt6.QtWidgets import QPushButton
from PyQt6.QtCore import QTimer

import styles


class EstopController:
    """Controls the Emergency Stop button state machine.
    Callback arguments:
        -log_fn(str)
            --Writes status messages to the terminal/log window.
        -trigger_fn()
            --Stops robot motion and switches the GUI to a safe state.
        -kill_fn()
            --Terminates any running robot processes.
        -reset_ui_fn()
            --Returns all GUI controls to their default state.
        -set_mode_fn(str)
            --Updates the application's current operating mode.
    """

    def __init__(
        self,
        estop_btn: QPushButton,
        log_fn,
        trigger_fn,
        kill_fn,
        reset_ui_fn,
        set_mode_fn,
        release_fn,
        hold_required_ms: int = 3000,
    ):
        # Reference to the GUI button
        self.estop_btn = estop_btn

        # Callback functions supplied by the main application
        self.log = log_fn
        self.trigger_fn = trigger_fn
        self.kill_fn = kill_fn
        self.reset_ui_fn = reset_ui_fn
        self.set_mode_fn = set_mode_fn
        self.release_fn = release_fn

        # True once the E-STOP has been activated
        self.latched = False

        # Indicates that the button is allowed to perform a reset
        self.reset_ready = False

        # Amount of time the button has been held (milliseconds)
        self.hold_time = 0

        # Required hold duration before releasing the E-STOP
        self.hold_required = hold_required_ms

        # Timer used to measure how long the button is held
        self.hold_timer = QTimer()

        # Update progress every 50 ms
        self.hold_timer.setInterval(50)

        # Each timeout updates the hold progress bar
        self.hold_timer.timeout.connect(self._update_hold)

        # Initialize the button appearance and signal connections
        self.setup_button()

    # =========================================================
    # Trigger / Release
    # =========================================================

    def trigger(self):
        """Activate the Emergency Stop."""

        # Inform the user
        self.log("E-STOP ACTIVATED")

        # Execute application-specific emergency actions
        self.trigger_fn()
        self.kill_fn()
        self.reset_ui_fn()

        # Update the application mode
        self.set_mode_fn("ESTOPPED")

        # Change the button appearance to indicate that it is latched
        self.estop_btn.setText("⛔ HOLD 3s TO RESET")
        self.estop_btn.setStyleSheet(styles.ESTOP_LATCHED)

        # Prevent immediate release
        self.latched = True
        self.reset_ready = True

    def release(self):
        """Release the Emergency Stop after a successful hold."""

        self.log("E-STOP RELEASED")
        self.set_mode_fn("ESTOP RELEASED")

        # Clear all internal state
        self.latched = False
        self.reset_ready = False

        # Stop timing and reset the progress
        self.hold_timer.stop()
        self.hold_time = 0
        
        self.release_fn()
        # Restore the button to its normal behavior
        self.setup_button()

        self.log(
            "System fully restored and reusable"
        )

    # =========================================================
    # Hold-to-reset timing
    # =========================================================

    def start_hold(self):
        """Begin timing when the user presses the button."""

        # Ignore presses unless we're currently estopped
        if not self.latched or not self.reset_ready:
            return

        # Restart the timer from zero
        self.hold_time = 0
        self.hold_timer.stop()
        self.hold_timer.start()

    def stop_hold(self):
        """Cancel the reset if the button is released too early."""

        if not self.latched:
            return

        # Stop measuring hold time
        self.hold_timer.stop()
        self.hold_time = 0

        # Restore the original button text
        if self.latched:
            self.estop_btn.setText("⛔ HOLD 3s TO RESET")

    def _update_hold(self):
        """Update hold progress every timer tick."""

        # Safety check in case the state changed unexpectedly
        if not self.latched or not self.reset_ready:
            self.hold_timer.stop()
            return

        # Add one timer interval (50 ms)
        self.hold_time += 50

        # Compute progress between 0.0 and 1.0
        progress = min(self.hold_time / self.hold_required, 1.0)

        # Convert progress into a 20-character ASCII progress bar
        bar = int(progress * 20)

        self.estop_btn.setText(
            "⛔ RESETTING [" + "*" * bar + "-" * (20 - bar) + "]"
        )

        # Gradually change the button color as progress increases
        self.estop_btn.setStyleSheet(
            styles.estop_hold_progress(progress)
        )

        # Automatically release once the hold requirement is met
        if self.hold_time >= self.hold_required:
            self.release()

    # =========================================================
    # Button wiring
    # =========================================================

    def setup_button(self):
        """Restore the E-STOP button to its default behavior.

        Existing signal connections are removed first to avoid
        duplicate callbacks if this function is called multiple
        times during the application's lifetime.
        """

        # Disconnect any existing signal handlers
        for signal_name in ("clicked", "pressed", "released"):
            try:
                getattr(self.estop_btn, signal_name).disconnect()
            except Exception:
                # Safe to ignore if nothing was connected
                pass

        # Clicking triggers the emergency stop
        self.estop_btn.clicked.connect(self.trigger)

        # Holding begins the reset countdown
        self.estop_btn.pressed.connect(self.start_hold)

        # Releasing early cancels the countdown
        self.estop_btn.released.connect(self.stop_hold)

        # Restore default appearance
        self.estop_btn.setStyleSheet(styles.ESTOP_DEFAULT)
        self.estop_btn.setText("⛔ E-STOP")

        # Ensure the button isn't visually stuck down
        self.estop_btn.setDown(False)
        self.estop_btn.setChecked(False)

        # Re-enable signal delivery
        self.estop_btn.blockSignals(False)
