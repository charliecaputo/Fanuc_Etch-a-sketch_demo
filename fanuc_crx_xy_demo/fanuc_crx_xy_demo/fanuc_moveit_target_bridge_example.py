#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class FanucMoveItTargetBridge(Node):
    def __init__(self):
        super().__init__('fanuc_moveit_target_bridge_example')
        self.create_subscription(PoseStamped, 'demo_target_pose', self.cb, 10)
        self.get_logger().info('Target bridge example started')

    def cb(self, msg: PoseStamped):
        self.get_logger().info(
            f'Received target pose in frame={msg.header.frame_id}: '
            f'x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}')
        # Replace this block with your real MoveIt2 or execution path.


def main():
    rclpy.init()
    node = FanucMoveItTargetBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
