#!/usr/bin/env python3

# ─────────────────────────────────────────────
# process_manager.py
# Owns every ROS subprocess the HMI launches
# (robot stack, manual init, encoder teleop,
# jog listener) plus the background log reader.
# Talks back to the UI only through the log_fn
# callback passed into the constructor.
# ─────────────────────────────────────────────

import subprocess
import os
import signal

from PyQt6.QtCore import QThread, pyqtSignal


class ProcessLogReader(QThread):
    line_received = pyqtSignal(str)

    def __init__(self, process):
        super().__init__()
        self.process = process
        self._running = True

    def run(self):
        if not self.process or not self.process.stdout:
            return

        while self._running:
            try:
                line = self.process.stdout.readline()
            except Exception:
                break

            if not line:
                self.msleep(10)
                continue

            self.line_received.emit(line.rstrip())

    def stop(self):
        self._running = False


class ProcessManager:
    """Starts/stops the ROS subprocesses used by the HMI and tracks
    their running state. `log_fn` is called with a single string
    argument any time something should be printed to the terminal
    widget.
    """

    def __init__(self, log_fn):
        self.log = log_fn

        # ── Process handles / state ──
        self.robot_process = None
        self.log_thread = None

        self.manual_init_process = None
        self.manual_init_running = False

        self.encoder_processes = []
        self.encoder_running = False

        self.jog_listener_process = None

    # =========================================================
    # Low-level helpers
    # =========================================================

    def launch_ros_process(self, command: str, capture_output: bool = False):
        kwargs = {"preexec_fn": os.setsid}

        if capture_output:
            kwargs.update({
                "stdout": subprocess.PIPE,
                "stderr": subprocess.STDOUT,
                "text": True,
                "bufsize": 1,
            })

        return subprocess.Popen(["bash", "-c", command], **kwargs)

    def terminate_process(self, proc):
        if not proc:
            return

        if proc.poll() is not None:
            return

        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            proc.terminate()

    # =========================================================
    # Robot stack (MoveIt)
    # =========================================================

    def start_robot_stack(self, cmd: str):
        """Launches the robot stack with output captured and streamed
        to `log_fn` via a background ProcessLogReader thread.
        Returns the Popen handle. Raises on failure.
        """
        self.robot_process = self.launch_ros_process(cmd, capture_output=True)

        self.log_thread = ProcessLogReader(self.robot_process)
        self.log_thread.line_received.connect(self.log)
        self.log_thread.start()

        return self.robot_process

    def kill_robot_stack(self):
        if self.robot_process is None:
            return

        if self.robot_process.poll() is None:
            self.log("Killing robot stack...")
            try:
                os.killpg(os.getpgid(self.robot_process.pid), signal.SIGTERM)

                cmd = (
                    "pkill -9 -f slider_gui_node & "
                    "pkill -9 -f servo_node & "
                    "pkill -9 -f move_group & "
                    "pkill -9 -f jog_listener_node "
                )
                subprocess.Popen(["bash", "-c", cmd], preexec_fn=os.setsid)

                if self.log_thread:
                    self.log_thread.stop()
                    self.log_thread.wait()
                    self.log_thread = None

            except Exception as e:
                self.log(f"SIGTERM failed: {e}")
                self.robot_process.terminate()

        self.robot_process = None

    # =========================================================
    # Manual init (home)
    # =========================================================

    def start_manual_init(self, cmd: str):
        self.manual_init_process = self.launch_ros_process(cmd)
        self.manual_init_running = True
        return self.manual_init_process

    def kill_manual_init(self):
        self.log("Killing manual init process...")
        self.terminate_process(self.manual_init_process)
        self.manual_init_process = None
        self.manual_init_running = False

    # =========================================================
    # Encoder teleop
    # =========================================================

    def start_encoder_teleop(self, cmd: str):
        proc = self.launch_ros_process(cmd)
        self.encoder_processes.append(proc)
        self.encoder_running = True
        return proc

    def kill_encoder_teleop(self):
        self.log("Killing encoder teleop processes...")
        for p in self.encoder_processes:
            self.terminate_process(p)
        self.encoder_processes = []
        self.encoder_running = False

    # =========================================================
    # Jog listener
    # =========================================================

    def start_jog_listener(self, cmd: str):
        self.log("Starting jog listener...")
        self.jog_listener_process = self.launch_ros_process(cmd)
        return self.jog_listener_process

    def kill_jog_listener(self):
        if not self.jog_listener_process:
            return

        if self.jog_listener_process.poll() is None:
            self.log("Stopping jog listener...")
            try:
                cmd = "pkill -9 -f jog_listener_node"
                subprocess.Popen(["bash", "-c", cmd], preexec_fn=os.setsid)
            except Exception:
                self.jog_listener_process.terminate()

        self.jog_listener_process = None

    # =========================================================
    # Shutdown
    # =========================================================

    def kill_all(self):
        """Used on window close — mirrors the original closeEvent,
        which did not stop the jog listener."""
        self.kill_robot_stack()
        self.kill_manual_init()
        self.kill_encoder_teleop()
