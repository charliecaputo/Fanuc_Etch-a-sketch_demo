#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pynput import keyboard

from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from std_msgs.msg import Float32MultiArray

import tf2_ros


class EncoderServo(Node):

    def __init__(self):
        super().__init__('encoder_servo')

        # =====================================================
        # Publisher
        # =====================================================
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )

        # =====================================================
        # Encoder state
        # =====================================================
        self.ENC_MIN = 0
        self.ENC_MAX = 4096

        self.encoder_x = 2048
        self.encoder_y = 2048

        self.keys_held = set()
        self.enc_step = 20

        # =====================================================
        # Workspace
        # =====================================================
        self.WS_X_MIN = 0.39
        self.WS_X_MAX = 0.66

        self.WS_Y_MIN = -0.23
        self.WS_Y_MAX = 0.34

        # Precompute scaling
        self.ws_x_range = self.WS_X_MAX - self.WS_X_MIN
        self.ws_y_range = self.WS_Y_MAX - self.WS_Y_MIN
        self.enc_scale = 1.0 / self.ENC_MAX

        # =====================================================
        # Controller
        # =====================================================
        self.kp = 5.0
        self.vmax = 0.5
        self.deadband = 0.002

        # =====================================================
        # TF
        # =====================================================
        self.base_frame = "base_link"
        self.ee_frame = "tool_link"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # Cached EE position
        self.current_x = None
        self.current_y = None

        # =====================================================
        # Encoder subscriber
        # =====================================================
        self.create_subscription(
            Float32MultiArray,
            '/encoder_x_mm',
            self.encoder_callback,
            10
        )

        # =====================================================
        # Keyboard listener
        # =====================================================
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

        # =====================================================
        # Reusable Twist message
        # =====================================================
        self.msg = TwistStamped()
        self.msg.header.frame_id = self.base_frame

        # =====================================================
        # Publish optimization
        # =====================================================
        self.last_vx = None
        self.last_vy = None

        # =====================================================
        # Activate MoveIt Servo
        # =====================================================
        self._activate_servo()

        # =====================================================
        # Timers
        # =====================================================

        # TF updates at 10 Hz
        self.tf_timer = self.create_timer(
            0.10,
            self.update_tf
        )

        # Control loop at 20 Hz
        self.control_timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            "Encoder Servo READY (optimized)"
        )

    # =========================================================
    # Encoder callback
    # =========================================================
    def encoder_callback(self, msg):

        if not msg.data:
            return

        angle_deg = float(msg.data[0])

        self.encoder_x = int(
            (angle_deg % 360.0) * (4096.0 / 360.0)
        )

    # =========================================================
    # Keyboard
    # =========================================================
    def on_press(self, key):
        try:
            self.keys_held.add(key.char)
        except Exception:
            pass

    def on_release(self, key):
        try:
            self.keys_held.discard(key.char)
        except Exception:
            pass

    def update_y(self):

        if 'j' in self.keys_held:
            self.encoder_y += self.enc_step

        if 'l' in self.keys_held:
            self.encoder_y -= self.enc_step

        self.encoder_y = max(
            self.ENC_MIN,
            min(self.ENC_MAX, self.encoder_y)
        )

    # =========================================================
    # TF cache update
    # =========================================================
    def update_tf(self):

        try:

            if not self.tf_buffer.can_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time()
            ):
                return

            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time()
            )

            self.current_x = tf.transform.translation.x
            self.current_y = tf.transform.translation.y

        except Exception:
            pass

    # =========================================================
    # Encoder -> workspace
    # =========================================================
    def encoder_to_workspace(self):

        target_x = (
            self.WS_X_MIN +
            self.encoder_x * self.enc_scale * self.ws_x_range
        )

        target_y = (
            self.WS_Y_MIN +
            self.encoder_y * self.enc_scale * self.ws_y_range
        )

        return target_x, target_y

    # =========================================================
    # Activate servo
    # =========================================================
    def _activate_servo(self):

        client = self.create_client(
            ServoCommandType,
            '/servo_node/switch_command_type'
        )

        if client.wait_for_service(timeout_sec=5.0):

            req = ServoCommandType.Request()
            req.command_type = 1

            client.call_async(req)

            self.get_logger().info(
                "Servo command type activated"
            )

        else:
            self.get_logger().warn(
                "Servo service unavailable"
            )

    # =========================================================
    # Control loop
    # =========================================================
    def control_loop(self):

        self.update_y()

        if self.current_x is None:
            return

        target_x, target_y = self.encoder_to_workspace()

        ex = target_x - self.current_x
        ey = target_y - self.current_y

        vx = 0.0
        vy = 0.0

        if abs(ex) > self.deadband:
            vx = max(
                -self.vmax,
                min(self.vmax, self.kp * ex)
            )

        if abs(ey) > self.deadband:
            vy = max(
                -self.vmax,
                min(self.vmax, self.kp * ey)
            )

        # Skip publish if command unchanged
        if (
            vx == self.last_vx and
            vy == self.last_vy
        ):
            return

        self.last_vx = vx
        self.last_vy = vy

        msg = self.msg

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.twist.linear.x = vx
        msg.twist.linear.y = vy

        self.twist_pub.publish(msg)


def main():
    rclpy.init()

    node = EncoderServo()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.listener.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
