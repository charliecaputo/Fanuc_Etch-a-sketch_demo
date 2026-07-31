#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

from piqt_interface.rmi_packets import (
    LinearMotionPacket,
    PositionData,
    ConfigurationData
)

from piqt_interface.fanuc_client import FanucRMIClient



class EtchSketchNode(Node):

    def __init__(self):

        super().__init__(
            "fanuc_etch_sketch"
        )


        #
        # FANUC workspace limits (mm)
        #
        self.WS_X_MIN = 420.0
        self.WS_X_MAX = 680.0

        self.WS_Y_MIN = -285.0
        self.WS_Y_MAX = 285.0


        #
        # AS5600 limits
        #
        self.ENC_MIN_DEG = 3.0
        self.ENC_MAX_DEG = 357.0


        #
        # RMI control
        #
        self.robot_ready = False

        # Only allow one command in flight
        self.max_outstanding = 2


        #
        # Initial robot position
        #
        self.target_x = (
            self.WS_X_MIN +
            self.WS_X_MAX
        ) / 2.0

        self.target_y = 0.0


        #
        # Last transmitted position
        #
        self.last_sent_x = None
        self.last_sent_y = None


        #
        # Ignore encoder jitter smaller than this
        #
        self.position_deadband = 1.0


        #
        # Drawing plane
        #
        self.robot_z = 0.0


        #
        # FANUC RMI
        #
        self.robot = FanucRMIClient(
            "10.69.17.244"
        )


        try:

            self.robot.initialize_robot()

            self.robot_ready = True

            self.get_logger().info(
                "Robot initialized."
            )


        except Exception as e:

            self.get_logger().error(
                f"Initialization failed: {e}"
            )



        #
        # Encoder subscriptions
        #
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


        #
        # FANUC command loop
        #
        self.motion_timer = self.create_timer(
            0.1,
            self.send_motion
        )



    #
    # ---------------------------------------------------------
    # Encoder callbacks
    # ---------------------------------------------------------
    #

    def encoder_x_callback(self, msg):

        if len(msg.data) < 1:
            return


        angle_deg = msg.data[0]


        self.target_x = self.map_range(
            angle_deg,
            self.ENC_MIN_DEG,
            self.ENC_MAX_DEG,
            self.WS_X_MIN,
            self.WS_X_MAX
        )



    def encoder_y_callback(self, msg):

        if len(msg.data) < 1:
            return


        angle_deg = msg.data[0]


        self.target_y = self.map_range(
            angle_deg,
            self.ENC_MIN_DEG,
            self.ENC_MAX_DEG,
            self.WS_Y_MIN,
            self.WS_Y_MAX
        )



    #
    # ---------------------------------------------------------
    # FANUC motion
    # ---------------------------------------------------------
    #

    def send_motion(self):

        if not self.robot_ready:
            return



        #
        # Prevent RMI flooding
        #
        if (
            self.robot.outstanding_commands
            >= self.max_outstanding
        ):
            return



        #
        # Ignore tiny movements
        #
        if self.last_sent_x is not None:

            dx = abs(
                self.target_x -
                self.last_sent_x
            )

            dy = abs(
                self.target_y -
                self.last_sent_y
            )


            if (
                dx < self.position_deadband
                and
                dy < self.position_deadband
            ):
                return



        #
        # Build FANUC packet
        #
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

        # safer for live encoder following
        packet.Speed = 100


        packet.TermType = "CNT"

        packet.TermValue = 100



        sequence = self.robot.send_motion(
            packet
        )


        self.last_sent_x = self.target_x
        self.last_sent_y = self.target_y


        self.get_logger().info(
            f"Sent {sequence}: "
            f"X={self.target_x:.1f}, "
            f"Y={self.target_y:.1f}, "
            f"queue={self.robot.outstanding_commands}"
        )



    #
    # ---------------------------------------------------------
    # Utilities
    # ---------------------------------------------------------
    #

    @staticmethod
    def map_range(
        value,
        in_min,
        in_max,
        out_min,
        out_max
    ):

        value = max(
            in_min,
            min(
                value,
                in_max
            )
        )


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

        node.get_logger().info(
            "Disconnecting from FANUC..."
        )

        node.robot.close()

        node.destroy_node()

        rclpy.shutdown()



if __name__ == "__main__":

    main()