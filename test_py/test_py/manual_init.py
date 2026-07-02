# ─────────────────────────────────────────────
# manual_init.py
#
# Robot startup pose initializer using MoveIt IK.
#
# Responsibilities:
#   • Wait for valid /joint_states input
#   • Query MoveIt IK service (/compute_ik)
#   • Compute a valid joint configuration for a fixed Cartesian pose
#   • Convert IK result into a JointTrajectory command
#   • Move robot safely to a predefined start pose
#
# NOTE:
#   • Uses MoveIt GetPositionIK service for inverse kinematics
#   • Requires a valid planning group ("manipulator")
#   • Assumes end-effector link is "flange"
#   • Publishes directly to joint_trajectory_controller
#   • Intended for system initialization only (run once at startup)
# ─────────────────────────────────────────────

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState, MoveItErrorCodes


class CartesianStartInitializer(Node):

    def __init__(self):
        super().__init__('cartesian_start_initializer')

        # -----------------------------
        # IK client
        # -----------------------------
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.ik_client.wait_for_service()

        # -----------------------------
        # trajectory publisher
        # -----------------------------
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # -----------------------------
        # joint state subscription
        # -----------------------------
        self.latest_joint_state = None
        self.joint_state_received = False

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10
        )

        self.done = False

        # wait for everything to come up
        self.create_timer(2.0, self.start_init)

        self.get_logger().info("IK initializer ready (waiting for joint_states)")

    # =========================================================
    # JOINT STATE CALLBACK
    # =========================================================
    def joint_state_cb(self, msg: JointState):
        self.latest_joint_state = msg
        self.joint_state_received = True

    # =========================================================
    # INIT START POSE
    # =========================================================
    def start_init(self):

        if self.done:
            return

        if not self.joint_state_received:
            self.get_logger().info("Waiting for /joint_states...")
            return

        self.done = True

        self.get_logger().info("Computing IK start pose...")

        # -----------------------------
        # Build proper RobotState seed
        # -----------------------------
        robot_state = RobotState()
        robot_state.joint_state = self.latest_joint_state

        # -----------------------------
        # IK request
        # -----------------------------
        ik = PositionIKRequest()

        ik.group_name = "manipulator"
        ik.ik_link_name = "flange"

        ik.robot_state = robot_state

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()

        # 🎯 Target start pose
        pose.pose.position.x = 0.54
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.15

        # SAFE orientation
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        ik.pose_stamped = pose
        ik.timeout.sec = 2

        req = GetPositionIK.Request()
        req.ik_request = ik

        future = self.ik_client.call_async(req)
        future.add_done_callback(self.ik_callback)

    # =========================================================
    # IK RESULT → TRAJECTORY
    # =========================================================
    def ik_callback(self, future):

        if future.exception():
            self.get_logger().error(f"IK service exception: {future.exception()}")
            return

        result = future.result()

        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error("IK failed for start pose")
            return

        js = result.solution.joint_state

        traj = JointTrajectory()
        traj.joint_names = js.name

        pt = JointTrajectoryPoint()
        pt.positions = js.position

        pt.time_from_start.sec = 5
        pt.time_from_start.nanosec = 0

        traj.points.append(pt)

        self.traj_pub.publish(traj)

        self.get_logger().info("Robot moved to start pose.")


def main():
    rclpy.init()
    node = CartesianStartInitializer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        #rclpy.shutdown()


if __name__ == "__main__":
    main()
