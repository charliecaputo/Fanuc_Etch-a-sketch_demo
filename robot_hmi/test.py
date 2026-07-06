#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from fanuc_msgs.msg import IOState


class F10Watcher(Node):
    def __init__(self):
        super().__init__('f10_watcher')

        self.create_subscription(
            IOState,
            '/fanuc_gpio_controller/io_state',
            self.callback,
            10
        )

    def callback(self, msg):

        for item in msg.values:
            if item.io_type.type == "DO" and item.index == 1:
                print(f"F10 = {item.value}")


def main():
    rclpy.init()
    node = F10Watcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
