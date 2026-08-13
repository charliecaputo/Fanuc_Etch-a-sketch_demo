#!/usr/bin/env python3

"""
test_rmi.py

Overview:
    This ROS 2 node converts two encoder inputs into X/Y position commands
    for a FANUC robot, creating an Etch-a-Sketch-style Cartesian motion
    interface.

    The node performs the following functions:

    1. Encoder Input:
       - Subscribes to encoder_x_mm and encoder_y_mm.
       - Reads the encoder values in degrees.
       - Maps the encoder ranges to the FANUC robot's defined X/Y workspace.

    2. Workspace Limiting:
       - Restricts the commanded X and Y positions to the defined FANUC
         workspace limits.
       - The robot operates on a fixed Z position, creating a 2D drawing plane.

    3. Motion Commands:
       - Periodically generates FANUC LinearMotionPacket commands.
       - Commands are sent through the /robot_command ROS 2 topic.
       - Motion is configured using FANUC RMI position, configuration, speed,
         and termination parameters.

    4. Position Deadband:
       - Small movements below the configured deadband are ignored.
       - When the target enters the deadband, a single CNT 0 hold command is
         sent to stop the robot at its current commanded position.
       - This prevents unnecessary repeated motion commands when the encoder
         is not moving significantly.

    5. Robot Responses:
       - Subscribes to /robot_response.
       - Converts incoming JSON responses into Python objects.
       - Logs the robot responses for monitoring and debugging.

    ROS Interfaces:
        Publishers:
            /robot_command
            Type: std_msgs/msg/String
            Purpose: Sends JSON-formatted FANUC RMI motion commands.

        Subscribers:
            /robot_response
            Type: std_msgs/msg/String
            Purpose: Receives responses from the FANUC robot connection node.

            encoder_x_mm
            Type: std_msgs/msg/Float32MultiArray
            Purpose: Provides the encoder input used to control robot X position.

            encoder_y_mm
            Type: std_msgs/msg/Float32MultiArray
            Purpose: Provides the encoder input used to control robot Y position.

    Robot Workspace:
        X: 400 mm to 600 mm
        Y: -200 mm to 200 mm
        Z: Fixed at 0.0 mm

    Encoder Range:
        3.0 degrees to 357.0 degrees

        Encoder values are clamped to this range before being mapped to the
        corresponding robot workspace.

    Motion Settings:
        Command update period: 0.1 seconds (10 Hz)
        Position deadband: 1.0 mm
        Linear speed: 200 mm/s
        Normal termination: CNT 100
        Deadband hold termination: CNT 0

    Shutdown:
        When the node is stopped, the motion node is destroyed and ROS 2
        is properly shut down.
"""

import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import Float32MultiArray, String

from piqt_interface.rmi_packets import (
    LinearMotionPacket,
    PositionData,
    ConfigurationData
)

class EtchSketchNode(Node):

    def __init__(self):

        super().__init__(
            "fanuc_etch_sketch"
        )

        # FANUC workspace limits
        self.WS_X_MIN = 400
        self.WS_X_MAX = 600
        self.WS_Y_MIN = -200
        self.WS_Y_MAX = 200

        # Encoder limits
        self.ENC_MIN_DEG = 3.0
        self.ENC_MAX_DEG = 357.0

        # Command control
        self.last_sent_x = None
        self.last_sent_y = None
        self.position_deadband = 1.0
        self.in_deadband = False

        # Drawing plane
        self.robot_z = 0.0

        # Current target
        self.target_x = ( self.WS_X_MIN + self.WS_X_MAX ) / 2.0
        self.target_y = 0.0

        #Send commands to robot_connection
        self.command_pub = self.create_publisher(
            String,
            "/robot_command",
            10
        )

        # Receive robot responses
        self.response_sub = self.create_subscription(
            String,
            "/robot_response",
            self.response_callback,
            10
        )

        # Encoder inputs
        self.create_subscription(
            Float32MultiArray,
            "encoder_x_mm",
            self.encoder_x_callback,
            10
        )

        self.create_subscription(
            Float32MultiArray,
            "encoder_y_mm",
            self.encoder_y_callback,
            10
        )

        # Motion timer
        self.motion_timer = self.create_timer(
            0.1,
            self.send_motion
        )

        self.get_logger().info(
            "EtchSketch motion node started"
        )

    # ---------------------------------------------------------
    # Robot responses
    # ---------------------------------------------------------
    def response_callback(self, msg):
        try:
            response = json.loads(
                msg.data
            )

            self.get_logger().info(
                f"Robot response: {response}"
            )

        except Exception as e:
            self.get_logger().error(
                f"Bad robot response: {e}"
            )

    # ---------------------------------------------------------
    # Encoder callbacks
    # --------------------------------------------------------
    def encoder_x_callback(self, msg):
        if len(msg.data) < 1:
            return

        self.target_x = self.map_range(
            msg.data[0],
            self.ENC_MIN_DEG,
            self.ENC_MAX_DEG,
            self.WS_X_MIN,
            self.WS_X_MAX
        )

    def encoder_y_callback(self, msg):

        if len(msg.data) < 1:
            return

        self.target_y = self.map_range(
            msg.data[0],
            self.ENC_MIN_DEG,
            self.ENC_MAX_DEG,
            self.WS_Y_MIN,
            self.WS_Y_MAX
        )

    # ---------------------------------------------------------
    # Send RMI motion command
    # ---------------------------------------------------------
    def send_motion(self):

        # Ignore tiny movements
        if self.last_sent_x is not None:
            dx = abs(
                self.target_x -
                self.last_sent_x
            )

            dy = abs(
                self.target_y -
                self.last_sent_y
            )

            if (dx < self.position_deadband and dy < self.position_deadband):
                # Only send one CNT 0 command when we first enter the deadband
                if not self.in_deadband:

                    packet = LinearMotionPacket()

                    packet.Configuration = ConfigurationData(
                        UToolNumber=1,
                        UFrameNumber=0,
                        Front=1,
                        Up=1,
                        Left=0,
                        Flip=1,
                        Turn4=0,
                        Turn5=0,
                        Turn6=0
                    )

                    packet.Position = PositionData(
                        X=self.last_sent_x,
                        Y=self.last_sent_y,
                        Z=self.robot_z,
                        W=180.0,
                        P=-90.0,
                        R=0.0
                    )

                    packet.SpeedType = "mmSec"
                    packet.Speed = 200

                    packet.TermType = "CNT"
                    packet.TermValue = 0

                    command = String()
                    command.data = packet.to_json()

                    self.command_pub.publish(command)

                    self.get_logger().info("Sent CNT 0 hold command")

                    self.in_deadband = True

                return

        # Build FANUC RMI packet
        packet = LinearMotionPacket()
        packet.Configuration = ConfigurationData(
            UToolNumber=1,
            UFrameNumber=0,
            Front=1,
            Up=1,
            Left=0,
            Flip=1,
            Turn4=0,
            Turn5=0,
            Turn6=0
        )

        packet.Position = PositionData(
            X=self.target_x,
            Y=self.target_y,
            Z=self.robot_z,
            W=180.0,
            P=-90.0,
            R=0.0
        )

        packet.SpeedType = "mmSec"
        packet.Speed = 200
        packet.TermType = "CNT"
        packet.TermValue = 100

        self.in_deadband = False

        # Convert packet to JSON
        command = String()
        command.data = packet.to_json()

        # Publish to robot_connection
        self.command_pub.publish(command)

        self.last_sent_x = self.target_x
        self.last_sent_y = self.target_y

        self.get_logger().info(
            f"Command sent X={self.target_x:.1f}, Y={self.target_y:.1f}"
        )

    # ---------------------------------------------------------
    # Utility
    # ---------------------------------------------------------

    @staticmethod
    def map_range(
        value,
        in_min,
        in_max,
        out_min,
        out_max
    ):

        value = max(in_min, min(value, in_max))

        return (
            (value - in_min)
            *
            (out_max - out_min)
            /
            (in_max - in_min)
            +
            out_min
        )

def main():
    rclpy.init()
    node = EtchSketchNode()
    try:
        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:
        node.get_logger().info("Stopping motion node")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
