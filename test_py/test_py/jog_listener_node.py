#!/usr/bin/env python3

# ─────────────────────────────────────────────
# jog_listener.py
#
# ROS2 MoveIt Servo jog controller node.
#
# Responsibilities:
#   • Receive jog commands from HMI (/hmi/jog_command)
#   • Receive speed scaling from UI (/hmi/jog_speed)
#   • Switch between Cartesian (twist) and joint jogging modes
#   • Convert UI commands into MoveIt Servo messages
#   • Publish TwistStamped and JointJog commands
#   • Enforce basic safety limits (e.g., minimum Z height)
#   • Maintain continuous motion via periodic update loop
#
# NOTE:
#   • Servo mode switching is handled via ServoCommandType service
#   • TF2 is used only for tool position monitoring (safety check)
#   • Cartesian and joint modes are mutually exclusive
#   • This node assumes MoveIt Servo is running and configured
# ─────────────────────────────────────────────

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import Float32

from control_msgs.msg import JointJog

class JogListener(Node):

    def __init__(self):
        super().__init__('jog_listener')

        # ----------------------------
        # Publisher to MoveIt Servo
        # ----------------------------
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )
        self.joint_pub = self.create_publisher(
            JointJog,
            '/servo_node/delta_joint_cmds',
            10
        )

        # ----------------------------
        # Jog state
        # ----------------------------        
        self.cartesian_direction = None
        self.joint_direction = None
        self.current_mode = None      # "twist" or "joint"
        
        self.servo_client = self.create_client(
            ServoCommandType,
            "/servo_node/switch_command_type"
        )

        self.servo_client.wait_for_service()
        
        # speed (meters/sec equivalent)
        #self.speed = 0.15
        # Maximum speeds
        self.max_linear_speed = 0.25      # m/s
        self.max_angular_speed = 1.5      # rad/s
        # Slider starts at 50%
        self.speed_scale = 0.5
        
        
        # ----------------------------
        # Subscriber from UI
        # ----------------------------
        self.create_subscription(
            String,
            '/hmi/jog_command',
            self.jog_callback,
            10
        )
        
        self.create_subscription(
            Float32,
            "/hmi/jog_speed",
            self.speed_callback,
            10
        )
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_z = None
        self.min_z = 0.05
        
        # ----------------------------
        # Control loop (keeps motion alive while held)
        # ----------------------------
        self.timer = self.create_timer(0.02, self.update)
        
        self._activate_servo()

        self.get_logger().info("Jog Listener READY")

    def _activate_servo(self):
        client = self.create_client(
            ServoCommandType,
            '/servo_node/switch_command_type'
        )

    def set_servo_mode(self, mode):
        if self.current_mode == mode:
            return

        req = ServoCommandType.Request()

        if mode == "twist":
            req.command_type = 1
        else:
            req.command_type = 0

        future = self.servo_client.call_async(req)

        def done(f):
            if f.result().success:
                self.current_mode = mode
                self.get_logger().info(f"Servo mode -> {mode}")
            else:
                self.get_logger().error("Failed to switch Servo mode")

        future.add_done_callback(done)

    # =========================================================
    # Receive UI commands
    # =========================================================
    def jog_callback(self, msg):
        cmd = msg.data.lower().strip()

        if cmd == "stop":
            self.cartesian_direction = None
            self.joint_direction = None
            return

        if cmd.startswith("+j") or cmd.startswith("-j"):

            self.set_servo_mode("joint")

            self.joint_direction = cmd
            self.cartesian_direction = None

        else:

            self.set_servo_mode("twist")

            self.cartesian_direction = cmd
            self.joint_direction = None
    
    def speed_callback(self, msg: Float32):
        self.speed_scale = max(0.0, min(1.0, msg.data))

        self.get_logger().info(
            f"Jog speed: {self.speed_scale*100:.0f}%"
        )

    # =========================================================
    # Convert direction → Twist
    # =========================================================
    def update(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                "tool_link",
                rclpy.time.Time()
            )
            self.current_z = transform.transform.translation.z
        except Exception:
            pass
        
        if self.current_mode == "joint" and self.joint_direction is not None:

            sign = 1.0 if self.joint_direction[0] == "+" else -1.0
            joint = int(self.joint_direction[2:])

            names = ["J1","J2","J3","J4","J5","J6"]

            msg = JointJog()
            msg.header.stamp = self.get_clock().now().to_msg()

            msg.joint_names = [names[joint-1]]
            msg.velocities = [sign * self.max_angular_speed * self.speed_scale]

            self.joint_pub.publish(msg)

            return
        
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "base_link"
        
        vx = vy = vz = wx = wy = wz = 0.0
        linear_speed = self.max_linear_speed * self.speed_scale
        angular_speed = self.max_angular_speed * self.speed_scale

        if self.cartesian_direction  == "+x":
            vx = linear_speed
        elif self.cartesian_direction  == "-x":
            vx = -linear_speed
            
        elif self.cartesian_direction  == "+y":
            vy = linear_speed
        elif self.cartesian_direction  == "-y":
            vy = -linear_speed
        
        elif self.cartesian_direction  == "+z":
            vz = linear_speed
        elif self.cartesian_direction  == "-z":
            if self.current_z is not None and self.current_z <= self.min_z:
                vz = 0.0
            else:
                vz = -linear_speed
        
        elif self.cartesian_direction  == "+roll":
            wx = angular_speed
        elif self.cartesian_direction  == "-roll":
            wx = -angular_speed
        
        elif self.cartesian_direction  == "+pitch":
            wy = angular_speed
        elif self.cartesian_direction  == "-pitch":
            wy = -angular_speed
        
        elif self.cartesian_direction  == "+yaw":
            wz = angular_speed
        elif self.cartesian_direction  == "-yaw":
            wz = -angular_speed
            
        if self.cartesian_direction is None:
            twist.twist.linear.x = 0.0
            twist.twist.linear.y = 0.0
            twist.twist.linear.z = 0.0
            twist.twist.angular.x = 0.0
            twist.twist.angular.y = 0.0
            twist.twist.angular.z = 0.0
            self.twist_pub.publish(twist)
            return
            

        twist.twist.linear.x = vx
        twist.twist.linear.y = vy
        twist.twist.linear.z = vz
        twist.twist.angular.x = wx
        twist.twist.angular.y = wy
        twist.twist.angular.z = wz

        self.twist_pub.publish(twist)

def main():
    rclpy.init()
    node = JogListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
