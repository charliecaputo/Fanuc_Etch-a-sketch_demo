#!/usr/bin/env python3

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

        #
        # Connect to FANUC
        #
        self.robot = FanucRMIClient(
            "10.69.17.244"
        )

        #
        # Initialize RMI
        #
        self.robot.initialize_robot()

        #
        # Publish every packet received from the robot
        #
        self.robot.add_packet_callback(
            self.robot_packet_callback
        )

        #
        # ROS interfaces
        #
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

        self.get_logger().info(
            "Robot connection node ready"
        )


    #
    # ---------------------------------------------------------
    # ROS -> FANUC
    # ---------------------------------------------------------
    #

    def command_callback(self, msg):

        try:

            command = json.loads(
                msg.data
            )

            self.robot.send_json(
                command
            )

        except Exception as e:

            self.get_logger().error(
                f"Command error: {e}"
            )


    #
    # ---------------------------------------------------------
    # FANUC -> ROS
    # ---------------------------------------------------------
    #

    def robot_packet_callback(
        self,
        packet
    ):

        try:

            ros_msg = String()

            ros_msg.data = json.dumps(
                packet
            )

            self.response_pub.publish(
                ros_msg
            )

        except Exception as e:

            self.get_logger().error(
                f"Publish error: {e}"
            )


    #
    # ---------------------------------------------------------
    # Shutdown
    # ---------------------------------------------------------
    #

    def destroy_node(self):

        self.get_logger().info(
            "Closing FANUC connection..."
        )

        try:

            self.robot.close()

        except Exception as e:

            self.get_logger().error(
                str(e)
            )

        super().destroy_node()


def main():

    rclpy.init()

    node = RobotConnection()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()#!/usr/bin/env python3

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

        #
        # Connect to FANUC
        #
        self.robot = FanucRMIClient(
            "10.69.17.244"
        )

        #
        # Initialize RMI
        #
        self.robot.initialize_robot()

        #
        # Publish every packet received from the robot
        #
        self.robot.add_packet_callback(
            self.robot_packet_callback
        )

        #
        # ROS interfaces
        #
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

        self.get_logger().info(
            "Robot connection node ready"
        )


    #
    # ---------------------------------------------------------
    # ROS -> FANUC
    # ---------------------------------------------------------
    #

    def command_callback(self, msg):

        try:

            command = json.loads(
                msg.data
            )

            self.robot.send_json(
                command
            )

        except Exception as e:

            self.get_logger().error(
                f"Command error: {e}"
            )


    #
    # ---------------------------------------------------------
    # FANUC -> ROS
    # ---------------------------------------------------------
    #

    def robot_packet_callback(
        self,
        packet
    ):

        try:

            ros_msg = String()

            ros_msg.data = json.dumps(
                packet
            )

            self.response_pub.publish(
                ros_msg
            )

        except Exception as e:

            self.get_logger().error(
                f"Publish error: {e}"
            )


    #
    # ---------------------------------------------------------
    # Shutdown
    # ---------------------------------------------------------
    #

    def destroy_node(self):

        self.get_logger().info(
            "Closing FANUC connection..."
        )

        try:

            self.robot.close()

        except Exception as e:

            self.get_logger().error(
                str(e)
            )

        super().destroy_node()


def main():

    rclpy.init()

    node = RobotConnection()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()