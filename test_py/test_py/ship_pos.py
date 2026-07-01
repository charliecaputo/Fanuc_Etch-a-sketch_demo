import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from std_msgs.msg import Bool

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
        
        self.shipping_joints = [
            -0.01629250888811002,
            0.3601057039025047,
            -1.1950727630225406,
            -0.19185376050363034,
            -0.0012348648759089413,
            0.20814446123443042
        ]

        self.joint_tolerance = 0.04  # radians (adjust if needed)
        self.direction = 1  # 1 = enter shipping, -1 = leave

        # -----------------------------
        # waypoint planning state
        # -----------------------------
        self.shipping_waypoints = [
            {
                "position": (0.54, 0.0, 0.15),
                "orientation": (0.0, 0.0, 0.0, 1.0),
            },
            {
                "position": (0.30, -0.07, 0.15),
                "orientation": (0.0, 0.41, 0.0, 1.0),
            },
            {
                "position": (0.17, -0.15, 0.05),
                "orientation": (0.0, 0.82, 0.0, 1.0),
            },
        ]

        self.waypoints = list(self.shipping_waypoints)

        self.current_waypoint = 0
        self.joint_solutions = []

        self.seed_state = None
        self.done = False

        #self.create_timer(2.0, self.start_init)
        self.create_subscription(
            Bool,
            "/ship_pose",
            self.ship_pose_cb,
            10
        )
        self.busy = False
        self.get_logger().info("Waiting for shipping position command...")

        self.get_logger().info("IK waypoint initializer ready")

    # =========================================================
    # JOINT STATE CALLBACK
    # =========================================================
    def joint_state_cb(self, msg: JointState):
        self.latest_joint_state = msg
        self.joint_state_received = True
        
    def ship_pose_cb(self, msg: Bool):
        # Ignore False messages
        if not msg.data:
            return

        # Ignore if already executing
        if self.busy:
            self.get_logger().warn("Already executing shipping motion.")
            return

        # Need a joint state before planning
        if not self.joint_state_received:
            self.get_logger().warn("No joint state received yet.")
            return

        self.busy = True

        self.current_waypoint = 0
        self.joint_solutions = []

        current_positions = self.latest_joint_state.position

        if self.is_at_shipping_pose(current_positions):
            self.get_logger().info("Leaving shipping position.")
            self.waypoints = list(reversed(self.shipping_waypoints))
        else:
            self.get_logger().info("Entering shipping position.")
            self.waypoints = list(self.shipping_waypoints)
        

        self.seed_state = RobotState()
        self.seed_state.joint_state = self.latest_joint_state

        self.solve_next_waypoint()
    
    def is_at_shipping_pose(self, joint_positions):
        if joint_positions is None or len(joint_positions) < 6:
            return False

        for actual, target in zip(joint_positions[:6], self.shipping_joints):
            if abs(actual - target) > self.joint_tolerance:
                return False

        return True

    # =========================================================
    # START
    # =========================================================
    def start_init(self):
        if self.done:
            return

        if not self.joint_state_received:
            self.get_logger().info("Waiting for /joint_states...")
            return

        self.done = True

        current_positions = self.latest_joint_state.position

        at_shipping = self.is_at_shipping_pose(current_positions)

        if at_shipping:
            self.get_logger().info("Robot at shipping pose → LEAVING shipping position")
            self.direction = -1
        else:
            self.get_logger().info("Robot NOT at shipping pose → ENTERING shipping position")
            self.direction = 1

        # Set waypoint order based on direction
        if self.direction == 1:
            self.waypoints = self.waypoints
        else:
            self.waypoints = list(reversed(self.waypoints))
        
        print(self.waypoints)
        self.current_waypoint = 0
        self.joint_solutions = []

        self.seed_state = RobotState()
        self.seed_state.joint_state = self.latest_joint_state

        self.solve_next_waypoint()

    # =========================================================
    # SOLVE ONE WAYPOINT
    # =========================================================
    def solve_ik(self, robot_state, waypoint):

        ik = PositionIKRequest()
        ik.group_name = "manipulator"
        ik.ik_link_name = "flange"
        ik.robot_state = robot_state

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        pose.pose.position.x = waypoint["position"][0]
        pose.pose.position.y = waypoint["position"][1]
        pose.pose.position.z = waypoint["position"][2]

        # Orientation
        pose.pose.orientation.x = waypoint["orientation"][0]
        pose.pose.orientation.y = waypoint["orientation"][1]
        pose.pose.orientation.z = waypoint["orientation"][2]
        pose.pose.orientation.w = waypoint["orientation"][3]

        ik.pose_stamped = pose
        ik.timeout.sec = 2

        req = GetPositionIK.Request()
        req.ik_request = ik

        return self.ik_client.call_async(req)

    # =========================================================
    # CHAIN IK CALLS
    # =========================================================
    def solve_next_waypoint(self):

        if self.current_waypoint >= len(self.waypoints):
            self.publish_trajectory()
            return

        wp = self.waypoints[self.current_waypoint]

        self.get_logger().info(
            f"Solving IK for waypoint {self.current_waypoint}"
        )

        future = self.solve_ik(self.seed_state, wp)
        future.add_done_callback(self.ik_callback)

    # =========================================================
    # IK CALLBACK
    # =========================================================
    def ik_callback(self, future):

        if future.exception():
            self.get_logger().error(str(future.exception()))
            return

        result = future.result()

        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"IK failed at waypoint {self.current_waypoint}"
            )
            return

        js = result.solution.joint_state

        self.joint_solutions.append(js.position)

        # seed next IK with current solution
        self.seed_state = RobotState()
        self.seed_state.joint_state = js

        self.current_waypoint += 1

        self.solve_next_waypoint()

    # =========================================================
    # PUBLISH TRAJECTORY
    # =========================================================
    def publish_trajectory(self):

        traj = JointTrajectory()
        traj.joint_names = self.seed_state.joint_state.name

        for i, positions in enumerate(self.joint_solutions):

            pt = JointTrajectoryPoint()
            pt.positions = positions
            
            # simple timing: 2 seconds per waypoint
            pt.time_from_start.sec = (i + 1) * 3

            traj.points.append(pt)

        self.traj_pub.publish(traj)

        self.get_logger().info("Initialization trajectory published.")
        self.busy = False


# =========================================================
# MAIN
# =========================================================
def main():
    rclpy.init()
    node = CartesianStartInitializer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
