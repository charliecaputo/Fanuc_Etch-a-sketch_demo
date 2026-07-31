#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

from rmi_packets import (
    LinearMotionPacket,
    PositionData,
    ConfigurationData
)

from fanuc_client import FanucRMIClient



class EtchSketchNode(Node):

    def __init__(self):

        super().__init__(
            "fanuc_etch_sketch"
        )


        #
        # Workspace limits (meters)
        #
        self.WS_X_MIN = 0.42
        self.WS_X_MAX = 0.68

        self.WS_Y_MIN = -0.285
        self.WS_Y_MAX = 0.285


        #
        # Current robot position (meters)
        #
        self.robot_x = (
            self.WS_X_MIN +
            self.WS_X_MAX
        ) / 2.0

        self.robot_y = 0.0


        #
        # Fixed drawing plane
        #
        self.robot_z = 0.250


        #
        # FANUC connection
        #
        self.robot = FanucRMIClient(
            "192.168.0.10"
        )


        #
        # Encoder subscriptions
        #
        self.create_subscription(
            Float32,
            "encoder_x_mm",
            self.encoder_x_callback,
            10
        )

        self.create_subscription(
            Float32,
            "encoder_y_mm",
            self.encoder_y_callback,
            10
        )


        self.sequence = 1


    def encoder_x_callback(self, msg):

        # encoder input is mm
        delta = msg.data / 1000.0

        self.robot_x += delta

        self.robot_x = self.clamp(
            self.robot_x,
            self.WS_X_MIN,
            self.WS_X_MAX
        )

        self.send_motion()



    def encoder_y_callback(self, msg):

        delta = msg.data / 1000.0

        self.robot_y += delta

        self.robot_y = self.clamp(
            self.robot_y,
            self.WS_Y_MIN,
            self.WS_Y_MAX
        )

        self.send_motion()



    def send_motion(self):

        packet = LinearMotionPacket()


        packet.SequenceID = self.sequence


        packet.Configuration = ConfigurationData(
            UToolNumber=1,
            UFrameNumber=1
        )


        #
        # Convert meters -> FANUC mm
        #
        packet.Position = PositionData(

            X=self.robot_x * 1000.0,

            Y=self.robot_y * 1000.0,

            Z=self.robot_z * 1000.0,

            W=180.0,
            P=0.0,
            R=90.0
        )


        packet.SpeedType = "mm/sec"
        packet.Speed = 100

        packet.TermType = "CNT"
        packet.TermValue = 0


        self.robot.send(
            packet
        )


        self.sequence += 1



    @staticmethod
    def clamp(value, minimum, maximum):

        return max(
            minimum,
            min(
                value,
                maximum
            )
        )



def main():

    rclpy.init()

    node = EtchSketchNode()

    rclpy.spin(node)

    node.robot.close()

    node.destroy_node()

    rclpy.shutdown()



if __name__ == "__main__":
    main()