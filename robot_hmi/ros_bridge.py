#!/usr/bin/env python3

# ─────────────────────────────────────────────
# ros_bridge.py
#
# ROS2 communication layer for the HMI.
#
# This class acts as a thin bridge between:
#   • PyQt GUI (UI thread)
#   • ROS2 node (background thread)
#
# Responsibilities:
#   • Publish jog commands, speed scaling, and mode signals
#   • Subscribe to robot state topics
#   • Emit Qt signals so the UI can react safely
#   • Run ROS spin loop in a dedicated thread
# ─────────────────────────────────────────────

import threading
import rclpy
from std_msgs.msg import String, Float64, Float32, Bool
from moveit_msgs.srv import ServoCommandType

from PyQt6.QtCore import pyqtSignal, QObject


class ROSBridge(QObject):
    """
    Qt-safe wrapper around a ROS2 node.

    This class ensures:
        • ROS callbacks never directly touch UI widgets
        • Communication is thread-safe via Qt signals
    """

    # UI-facing signals
    mode_changed = pyqtSignal(str)
    connection_changed = pyqtSignal(bool)
    ship_state_changed = pyqtSignal(bool)

    def __init__(self):
        super().__init__()

        # Initialize ROS2 client library
        rclpy.init()

        # Flag used to prevent publishing after shutdown
        self.alive = True

        # Create ROS node for HMI communication
        self.node = rclpy.create_node('hmi_node')

        # =========================================================
        # Publishers
        # =========================================================

        # Publishes whether robot should enter "ship pose" mode
        self.ship_pose_pub = self.node.create_publisher(
            Bool, "/ship_pose", 10
        )

        # Publishes jog commands (+x, -y, stop, etc.)
        self.jog_pub = self.node.create_publisher(
            String, '/hmi/jog_command', 10
        )

        # Publishes speed scaling factor (0.0–1.0)
        self.speed_pub = self.node.create_publisher(
            Float32, '/hmi/jog_speed', 10
        )

        # =========================================================
        # Subscribers
        # =========================================================

        # Robot state: whether robot is in "ship pose"
        self.ship_state_sub = self.node.create_subscription(
            Bool, "/in_ship_pose", self.ship_state_cb, 10
        )

        # Internal state
        self.in_ship_pose = None
        self.speed = 0.5
        self._connected = False

        # Start ROS spinning in background thread
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    # =========================================================
    # ROS spin thread
    # =========================================================

    def _spin(self):
        """Background ROS event loop."""
        rclpy.spin(self.node)

    # =========================================================
    # Service / action helpers
    # =========================================================

    def _activate_servo(self):
        """
        Switch MoveIt Servo into correct command mode.

        Runs in a background thread so UI is never blocked.
        """
        def _do():
            client = self.node.create_client(
                ServoCommandType,
                '/servo_node/switch_command_type'
            )

            if client.wait_for_service(timeout_sec=10.0):
                req = ServoCommandType.Request()
                req.command_type = 1
                client.call_async(req)

        threading.Thread(target=_do, daemon=True).start()

    # =========================================================
    # Publishers (UI → ROS)
    # =========================================================

    def send_mode(self, mode: str):
        """
        Publish a mode string to ROS.

        NOTE: This references self.mode_pub, which is not defined
        in this snippet (likely exists elsewhere or is legacy).
        """
        if not self.alive:
            return

        try:
            msg = String()
            msg.data = mode
            self.mode_pub.publish(msg)
        except Exception as e:
            print(f"Publish failed (ignored): {e}")

    def send_jog(self, direction: str):
        """
        Publish jog command.

        Valid values:
            '+x', '-x', '+y', '-y', '+z', '-z', 'stop'
        """
        if not self.alive:
            return

        try:
            msg = String()
            msg.data = direction
            self.jog_pub.publish(msg)
        except Exception as e:
            print(f"Jog publish failed (ignored): {e}")

    def send_speed(self, value):
        """
        Publish speed scaling factor.

        Args:
            value (float): expected range [0.0, 1.0]
        """
        if not self.alive:
            return

        try:
            self.speed = value
            msg = Float32()
            msg.data = value
            self.speed_pub.publish(msg)
        except Exception as e:
            print(f"Speed publish failed (ignored): {e}")

    def publish_ship_pose(self, value: bool = True):
        """
        Request robot to enter/exit ship pose mode.
        """
        msg = Bool()
        msg.data = value
        self.ship_pose_pub.publish(msg)

    # =========================================================
    # Subscribers (ROS → UI)
    # =========================================================

    def ship_state_cb(self, msg):
        """
        Callback for /in_ship_pose topic.

        Updates internal state and notifies UI via signal.
        """
        self.in_ship_pose = msg.data
        self.ship_state_changed.emit(msg.data)

    def _mode_cb(self, msg):
        """
        (Unused in current snippet)
        Would handle mode updates and connection state.
        """
        self._connected = True
        self.connection_changed.emit(True)
        self.mode_changed.emit(msg.data)

    # =========================================================
    # Shutdown
    # =========================================================

    def shutdown(self):
        """Clean shutdown of ROS node and thread."""
        if not self.alive:
            return

        self.alive = False
        rclpy.shutdown()
