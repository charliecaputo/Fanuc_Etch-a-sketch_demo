#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped


class XYTargetNode(Node):
    def __init__(self):
        super().__init__('xy_target_node')
        self.declare_parameter('base_frame', 'world')
        self.declare_parameter('target_topic', 'demo_target_pose')
        self.declare_parameter('z_mm', 250.0)
        self.declare_parameter('yaw_deg', 0.0)
        self.declare_parameter('x_offset_mm', 400.0)
        self.declare_parameter('y_offset_mm', 0.0)
        self.declare_parameter('publish_target', True)

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.target_topic = str(self.get_parameter('target_topic').value)
        self.z_mm = float(self.get_parameter('z_mm').value)
        self.yaw_deg = float(self.get_parameter('yaw_deg').value)
        self.x_offset = float(self.get_parameter('x_offset_mm').value)
        self.y_offset = float(self.get_parameter('y_offset_mm').value)
        self.publish_target = bool(self.get_parameter('publish_target').value)

        self.subscription = self.create_subscription(Float32MultiArray, 'encoder_xy_mm', self.cb, 10)
        self.pose_pub = self.create_publisher(PoseStamped, self.target_topic, 10)
        self.get_logger().info('xy_target_node started')

    def yaw_to_quaternion(self, yaw_rad: float):
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)
        return 0.0, 0.0, qz, qw

    def cb(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warn('encoder_xy_mm must contain [x_deg, y_deg, x_mm, y_mm]')
            return

        _, _, x_mm, y_mm = msg.data
        target_x_mm = self.x_offset + x_mm
        target_y_mm = self.y_offset + y_mm
        target_z_mm = self.z_mm

        self.get_logger().info(
            f'Encoder target -> x_mm={target_x_mm:.2f}, y_mm={target_y_mm:.2f}, z_mm={target_z_mm:.2f}')

        if not self.publish_target:
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = target_x_mm / 1000.0
        pose.pose.position.y = target_y_mm / 1000.0
        pose.pose.position.z = target_z_mm / 1000.0
        qx, qy, qz, qw = self.yaw_to_quaternion(math.radians(self.yaw_deg))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pose_pub.publish(pose)


def main():
    rclpy.init()
    node = XYTargetNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
