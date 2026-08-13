#!/usr/bin/env python3

"""
robot_connection.py

Overview:
    This ROS 2 node provides a communication bridge between ROS 2 and a
    FANUC robot using the FANUC RMI client.

    The node performs two main functions:

    1. ROS 2 -> FANUC:
       - Subscribes to the /robot_command topic.
       - Expects commands as JSON-formatted strings.
       - Converts the received string into a Python dictionary/object.
       - Sends the command to the FANUC robot through the RMI connection.

    2. FANUC -> ROS 2:
       - Registers a callback with the FANUC RMI client.
       - Receives packets/data from the robot.
       - Converts the received data into a JSON-formatted string.
       - Publishes the data on the /robot_response topic.

    ROS Interfaces:
        Subscriber:
            /robot_command
            Type: std_msgs/msg/String
            Purpose: Receives JSON commands intended for the FANUC robot.

        Publisher:
            /robot_response
            Type: std_msgs/msg/String
            Purpose: Publishes JSON-formatted responses/packets received
                     from the FANUC robot.

    Robot Connection:
        The node connects to the FANUC controller at:
            10.69.17.244

        The FANUC RMI connection is initialized when the node starts and
        properly closed when the node shuts down.

    Error Handling:
        Errors while processing commands or publishing robot responses are
        caught and reported through the ROS 2 logger.

    Shutdown:
        When the ROS 2 node is stopped, the FANUC RMI connection is closed
        before the node is destroyed.
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from piqt_interface.fanuc_client import FanucRMIClient


class RobotConnection(Node):
    def __init__(self):
        super().__init__(
            "robot_connection"
        )

        # Connect to FANUC
        self.robot = FanucRMIClient("10.69.17.244")

        # Initialize RMI
        self.robot.initialize_robot()

        # Publish every packet received from the robot
        self.robot.add_packet_callback(
            self.robot_packet_callback
        )

        # ROS interfaces
        self.command_sub = self.create_subscription(
            String,
            "/robot_command",
            self.command_callback,
            10
        )

        self.response_pub = self.create_publisher(
            String,
            "/robot_response",
            10
        )

        self.get_logger().info("Robot connection node ready")

    # ---------------------------------------------------------
    # ROS -> FANUC
    # ---------------------------------------------------------
    def command_callback(self, msg):
        try:
            command = json.loads(msg.data)
            self.robot.send_json(command)

        except Exception as e:
            self.get_logger().error(f"Command error: {e}")

    # ---------------------------------------------------------
    # FANUC -> ROS
    # ---------------------------------------------------------
    def robot_packet_callback(self, packet):
        try:
            ros_msg = String()
            ros_msg.data = json.dumps(packet)

            self.response_pub.publish(ros_msg)

        except Exception as e:
            self.get_logger().error(f"Publish error: {e}")

    # ---------------------------------------------------------
    # Shutdown
    # ---------------------------------------------------------
    def destroy_node(self):
        self.get_logger().info("Closing FANUC connection...")

        try:
            self.robot.close()

        except Exception as e:
            self.get_logger().error(str(e))

        super().destroy_node()


def main():
    rclpy.init()
    node = RobotConnection()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
