import rclpy
from rclpy.node import Node

import copy

from pynput import keyboard

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from trajectory_msgs.msg import JointTrajectory

from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import RobotState


class EtchASketchCartesian(Node):

    def __init__(self):

        super().__init__('etch_a_sketch_cartesian')

        # -----------------------------
        # Cartesian planner client
        # -----------------------------
        self.cartesian_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path'
        )
        self.cartesian_client.wait_for_service()

        # -----------------------------
        # Publisher
        # -----------------------------
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # -----------------------------
        # Joint state (REAL robot state)
        # -----------------------------
        self.current_joint_state = JointState()

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # -----------------------------
        # Workspace
        # -----------------------------
        self.X_MIN = 0.39
        self.X_MAX = 0.68
        self.Y_MIN = -0.295
        self.Y_MAX = 0.295

        # -----------------------------
        # Encoder state
        # -----------------------------
        self.encoder_x = 2048
        self.encoder_y = 2048
        self.last_encoder_x = 2048
        self.last_encoder_y = 2048

        self.encoder_scale = 0.0002

        # -----------------------------
        # Virtual pen pose
        # -----------------------------
        self.x = 0.54
        self.y = 0.0
        self.z = 0.15

        # -----------------------------
        # Waypoints
        # -----------------------------
        self.waypoints = []
        self.cartesian_busy = False

        # -----------------------------
        # Keyboard
        # -----------------------------
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        # -----------------------------
        # Timers
        # -----------------------------
        self.create_timer(0.005, self.integrate_motion)
        self.create_timer(0.5, self.plan_cartesian_path)

        self.get_logger().info("Cartesian Etch-A-Sketch READY")

    # =========================================================
    # Joint state callback
    # =========================================================
    def joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg

    # =========================================================
    # Keyboard input
    # =========================================================
    def on_press(self, key):

        try:
            k = key.char
        except:
            return

        if k == 'i':
            self.encoder_x += 50
        elif k == 'k':
            self.encoder_x -= 50
        elif k == 'j':
            self.encoder_y += 50
        elif k == 'l':
            self.encoder_y -= 50

        self.encoder_x = max(0, min(4095, self.encoder_x))
        self.encoder_y = max(0, min(4095, self.encoder_y))

    # =========================================================
    # Integrate encoder → virtual pen
    # =========================================================
    def integrate_motion(self):

        dx = self.encoder_x - self.last_encoder_x
        dy = self.encoder_y - self.last_encoder_y

        # if (dx == 0 and dy == 0):
        #     self.get_logger().info(
        #         f"encoder_x= {self.encoder_x:.2f} || encoder_y= {self.encoder_y}"
        #     )

        self.last_encoder_x = self.encoder_x
        self.last_encoder_y = self.encoder_y

        self.x += dx * self.encoder_scale
        self.y += dy * self.encoder_scale

        self.x = max(self.X_MIN, min(self.X_MAX, self.x))
        self.y = max(self.Y_MIN, min(self.Y_MAX, self.y))

        p = Pose()
        p.position.x = self.x
        p.position.y = self.y
        p.position.z = self.z
        p.orientation.w = 1.0

        self.waypoints.append(copy.deepcopy(p))

        if len(self.waypoints) > 15:
            self.waypoints.pop(0)

    # =========================================================
    # Cartesian planning
    # =========================================================
    def plan_cartesian_path(self):

        if self.cartesian_busy:
            return

        if len(self.waypoints) < 5:
            return

        req = GetCartesianPath.Request()

        req.header.frame_id = "base_link"

        req.group_name = "manipulator"
        req.link_name = "flange"

        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.prismatic_jump_threshold = 0.0
        req.revolute_jump_threshold = 0.0

        req.avoid_collisions = False

        # -----------------------------
        # IMPORTANT: inject real start state
        # -----------------------------
        start_state = RobotState()
        start_state.joint_state = self.current_joint_state
        req.start_state = start_state

        req.waypoints = list(self.waypoints)

        self.cartesian_busy = True

        future = self.cartesian_client.call_async(req)
        future.add_done_callback(self.cartesian_response)

    # =========================================================
    # Response
    # =========================================================
    def cartesian_response(self, future):

        self.cartesian_busy = False

        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(str(e))
            return

        # self.get_logger().info(
        #     f"fraction={result.fraction:.2f}, points={len(result.solution.joint_trajectory.points)}"
        # )

        if result.error_code.val != 1:
            return

        self.traj_pub.publish(result.solution.joint_trajectory)

        self.waypoints.clear()


def main():
    rclpy.init()
    node = EtchASketchCartesian()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
