# ─────────────────────────────────────────────
# servo_control.py
#
# ROS2 MoveIt Servo encoder-based Cartesian controller.
#
# Responsibilities:
#   • Convert encoder values into workspace targets
#   • Track end-effector pose via TF2
#   • Compute Cartesian error (x/y) to target position
#   • Apply proportional control (P-controller)
#   • Publish TwistStamped commands to MoveIt Servo
#   • Support keyboard-assisted secondary encoder axis control
#   • Maintain continuous low-latency control loop
#
# NOTE:
#   • Uses MoveIt Servo switch_command_type service for activation
#   • Assumes base_link → tool_link TF is available
#   • Encoder values are mapped into a fixed workspace region
#   • Designed for real-time closed-loop Cartesian control
# ─────────────────────────────────────────────

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

import socket




class EncoderServo(Node):

    def __init__(self):
        super().__init__('encoder_servo')

        # =====================================================
        # Publisher
        # =====================================================
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            1
        )

        # =====================================================
        # Encoder state
        # =====================================================
        self.DEG_MIN = 3
        self.DEG_MAX = 357
        self.deg_range = self.DEG_MAX - self.DEG_MIN

        self.encoder_x = None
        self.encoder_y = None

        # =====================================================
        # Workspace
        # =====================================================
        self.WS_X_MIN = 0.42
        self.WS_X_MAX = 0.68

        self.WS_Y_MIN = -0.285
        self.WS_Y_MAX = 0.285

        # Precompute scaling
        self.ws_x_range = self.WS_X_MAX - self.WS_X_MIN
        self.ws_y_range = self.WS_Y_MAX - self.WS_Y_MIN
        
        # =====================================================
        # Controller
        # =====================================================
        self.kp = 2
        self.vmax = 0.25 #m/s
        self.deadband = 0.005
        self.max_accel = 0.5     # m/s²
        self.dt = 0.01
        self.max_delta = self.max_accel * self.dt
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        
        # Filter coefficient (0 < alpha <= 1)
        self.tf_alpha = 0.05

        self.filtered_x = None
        self.filtered_y = None
        
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
            self.encoder_x_callback,
            1
        )
        
        self.create_subscription(
            Float32MultiArray,
            '/encoder_y_mm',
            self.encoder_y_callback,
            1
        )

        # =====================================================
        # Reusable Twist message
        # =====================================================
        self.msg = TwistStamped()
        self.msg.header.frame_id = self.base_frame

        # =====================================================
        # Publish optimization
        # =====================================================
        self.last_vx = 0
        self.last_vy = 0

        # =====================================================
        # Activate MoveIt Servo
        # =====================================================
        self._activate_servo()

        # =====================================================
        # Timers
        # =====================================================

        # TF updates at 50 Hz
       # self.tf_timer = self.create_timer(
       #     0.01,
       #     self.update_tf
       # )

        # Control loop at 50 Hz
        self.control_timer = self.create_timer(
            0.01,
            self.control_loop
        )

        self.get_logger().info(
            "Encoder Servo READY"
        )

    # =========================================================
    # Encoder callback
    # =========================================================
    def encoder_x_callback(self, msg):

        if not msg.data:
            return

        self.encoder_x = float(msg.data[0])
        
        
    def encoder_y_callback(self, msg):

        if not msg.data:
            return

        self.encoder_y = float(msg.data[0])
                
    def limit_acceleration(self, desired, current):        
        delta = desired - current

        if delta > self.max_delta:
            delta = self.max_delta
        elif delta < -self.max_delta:
            delta = -self.max_delta

        return current + delta      
    

    # =========================================================
    # Encoder -> workspace
    # =========================================================
    def encoder_to_workspace(self):
        target_x = (
            self.WS_X_MIN +
            (self.encoder_x / self.deg_range) * self.ws_x_range
        )

        target_y = (
            self.WS_Y_MIN +
            (self.encoder_y / self.deg_range) * self.ws_y_range
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

            x = tf.transform.translation.x
            y = tf.transform.translation.y

            if self.filtered_x is None:
                self.filtered_x = x
                self.filtered_y = y
            else:
                self.filtered_x += self.tf_alpha * (x - self.filtered_x)
                self.filtered_y += self.tf_alpha * (y - self.filtered_y)
            
            self.current_x = self.filtered_x
            self.current_y = self.filtered_y

        except Exception:
            pass

        if (
            self.current_x is None or
            self.current_y is None or
            self.encoder_x is None or
            self.encoder_y is None
        ):
            return
        
        target_x, target_y = self.encoder_to_workspace()
        
        ex = target_x - self.current_x
        ey = target_y - self.current_y
        
        
        vx = 0.0
        vy = 0.0
        
        desired_vx = 0.0
        desired_vy = 0.0    
        if abs(ex) > self.deadband:
            desired_vx = max(
                -self.vmax,
                min(self.vmax, self.kp * ex)
            )

        if abs(ey) > self.deadband:
            desired_vy = max(
                -self.vmax,
                min(self.vmax, self.kp * ey)
            )
        
        # set accel
        self.cmd_vx = self.limit_acceleration(
            desired_vx,
            self.cmd_vx
        )
        #set accel
        self.cmd_vy = self.limit_acceleration(
            desired_vy,
            self.cmd_vy
        )
        
        vx = self.cmd_vx
        vy = self.cmd_vy
        
#        print(
#            f"target=({target_x:.3f}, {target_y:.3f}) "
#            f"current=({self.current_x:.3f}, {self.current_y:.3f}) "
#            f"error=({ex:.3f}, {ey:.3f}) "
#            f"cmd=({vx:.3f}, {vy:.3f})\n"
#        )
        
        #print(self.current_y)

        # Skip publish if command unchanged
#        if (
#            abs(vx - self.last_vx) < 1e-4 and
#            abs(vy - self.last_vy) < 1e-4
#        ):
#            vx = 0.0
#            vy = 0.0

        self.last_vx = vx
        self.last_vy = vy

        msg = self.msg

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.twist_pub.publish(msg)


def main():
    rclpy.init()

    node = EncoderServo()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
