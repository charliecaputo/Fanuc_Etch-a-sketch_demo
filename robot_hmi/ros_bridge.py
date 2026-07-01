#!/usr/bin/env python3

# ─────────────────────────────────────────────
# ros_bridge.py
# ROS2 bridge — background thread
# ─────────────────────────────────────────────

import threading
import rclpy
from std_msgs.msg import String, Float64, Float32
from moveit_msgs.srv import ServoCommandType

from PyQt6.QtCore import pyqtSignal, QObject


class ROSBridge(QObject):

    mode_changed = pyqtSignal(str)
    connection_changed = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        rclpy.init()

        self.alive = True

        self.node = rclpy.create_node('hmi_node')

        # ── Mode pub/sub ──
        self.mode_pub = self.node.create_publisher(
            String, '/hmi/mode_command', 10
        )
        self.node.create_subscription(
            String, '/hmi/current_mode', self._mode_cb, 10
        )

        # ── Jog direction publisher ──
        # Publishes a string like "+x", "-y", "+z", "stop"
        self.jog_pub = self.node.create_publisher(
            String, '/hmi/jog_command', 10
        )

        # ── Speed scaling publisher ──
        # Publishes Float32 in range [0.0, 1.0] to /speed_scaling_factor
        self.speed_pub = self.node.create_publisher(
            Float32, '/hmi/jog_speed', 10
        )

        self._connected = False
        self.speed = 0.5
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        rclpy.spin(self.node)

    def _mode_cb(self, msg):
        self._connected = True
        self.connection_changed.emit(True)
        self.mode_changed.emit(msg.data)

    def _activate_servo(self):
        def _do():
            client = self.node.create_client(
                ServoCommandType, '/servo_node/switch_command_type'
            )
            if client.wait_for_service(timeout_sec=10.0):
                req = ServoCommandType.Request()
                req.command_type = 1
                client.call_async(req)
        threading.Thread(target=_do, daemon=True).start()

    def send_mode(self, mode: str):
        if not self.alive:
            print("ROS is shut down; ignoring publish")
            return
        try:
            msg = String()
            msg.data = mode
            self.mode_pub.publish(msg)
        except Exception as e:
            print(f"Publish failed (ignored): {e}")

    def send_jog(self, direction: str):
        """
        Publish a jog direction command.
        Values: '+x' | '-x' | '+y' | '-y' | '+z' | '-z' | 'stop'
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
        Publish speed scaling factor [0.0, 1.0] to /speed_scaling_factor.
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

    def shutdown(self):
        if not self.alive:
            return
        self.alive = False
        rclpy.shutdown()
