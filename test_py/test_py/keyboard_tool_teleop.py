import rclpy
from rclpy.node import Node
import math

#insure pynput is installed: sudo apt install python3-pynput
#this wont be needed in the future.
from pynput import keyboard

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped


class IKTeleop(Node):
    def __init__(self):
        super().__init__('ik_keyboard_teleop')

        #start inverse kinematics service
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.ik_client.wait_for_service()

        #trajectory publisher
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        #set step size and start pos for encoder value and robot
        self.encoder_x = 2048
        self.encoder_y = 2048
        self.encoder_step = 50

        #self.target_x = 0.54
        #self.target_y = 0.0

        # robot's commanded position
        self.current_x = 0.54
        self.current_y = 0.0

        # encoder desired position
        self.desired_x = 0.54
        self.desired_y = 0.0

        # max Cartesian motion per update (meters)
        self.max_move = 0.005    # 10 mm

        # vairables to ensure ik request aren't flooded
        self.ik_busy = False
        self.last_encoder_x = None
        self.last_encoder_y = None
        
        self.last_joint_state = None

        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        # IMPORTANT: ROS timer handles execution (no blocking callbacks)
        self.timer = self.create_timer(0.05, self.process_motion)# old = 0.2

        self.get_logger().info("IK Teleop READY")

    def on_press(self, key):
        """
        function for taking key presses and translating to pos
        """
        try:
            k = key.char
        except:
            return

        if k == 'i':
            self.encoder_x = min(4095, self.encoder_x + self.encoder_step)

        elif k == 'k':
            self.encoder_x = max(0, self.encoder_x - self.encoder_step)

        elif k == 'j':
            self.encoder_y = min(4095, self.encoder_y + self.encoder_step)

        elif k == 'l':
            self.encoder_y = max(0, self.encoder_y - self.encoder_step)
        
        print("encoder x: ", self.encoder_x, "\n encoder y: ", self.encoder_y )

    def process_motion(self):
        #check for flooding ik service
        # if (
        #     self.encoder_x == self.last_encoder_x and
        #     self.encoder_y == self.last_encoder_y
        # ):
        #     return

        if self.ik_busy:
            return

        self.last_encoder_x = self.encoder_x
        self.last_encoder_y = self.encoder_y
        
        X_MIN = 0.39
        X_MAX = 0.68

        Y_MIN = -0.295
        Y_MAX = 0.295

        #set target position mapping encoder to pos to min max of working area
        # self.target_x = (
        #     X_MIN +
        #     (self.encoder_x / 4095.0) * (X_MAX - X_MIN)
        # )

        # #set target position mapping encoder to pos to min max of working area
        # self.target_y = (
        #     Y_MIN +
        #     (self.encoder_y / 4095.0) * (Y_MAX - Y_MIN)
        # )

        self.desired_x = (
            X_MIN +
            (self.encoder_x / 4095.0) * (X_MAX - X_MIN)
        )

        self.desired_y = (
            Y_MIN +
            (self.encoder_y / 4095.0) * (Y_MAX - Y_MIN)
        )

        error_x = self.desired_x - self.current_x
        error_y = self.desired_y - self.current_y
        distance = math.sqrt(error_x**2 + error_y**2)
        if distance < 0.001:
            return
        step = min(self.max_move, distance)

        self.current_x += step * error_x / distance
        self.current_y += step * error_y / distance

        #called safely in ROS thread
        req = GetPositionIK.Request()

        ik_req = PositionIKRequest()
        ik_req.group_name = "manipulator"
        ik_req.ik_link_name = "flange"

        #update position message
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        #pose.pose.position.x = self.target_x
        #pose.pose.position.y = self.target_y
        pose.pose.position.x = self.current_x
        pose.pose.position.y = self.current_y
        pose.pose.position.z = 0.15
        pose.pose.orientation.w = 1.0

        ik_req.pose_stamped = pose
        ik_req.timeout.sec = 1
        
        if self.last_joint_state is not None:
            ik_req.robot_state.joint_state = self.last_joint_state
        
        #request to solve ik
        req.ik_request = ik_req

        self.ik_busy = True
        
        #set future to this solved ik
        future = self.ik_client.call_async(req)

        def callback(fut):
            """
            takes future if its solveable and moves the 
            robot based on the solved kinematics
            """
            self.ik_busy = False
            result = fut.result()

            if not result or result.error_code.val != 1:
                self.get_logger().warn("IK failed")
                return

            js = result.solution.joint_state
            
            self.last_joint_state = js
            
            traj = JointTrajectory()
            traj.joint_names = js.name

            point = JointTrajectoryPoint()
            point.positions = js.position
            point.time_from_start.sec = 1

            traj.points.append(point)

            self.traj_pub.publish(traj)

        future.add_done_callback(callback)

def main():
    rclpy.init()
    node = IKTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()