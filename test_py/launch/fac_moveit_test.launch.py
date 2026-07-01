# SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
# SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch.actions import TimerAction

from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

import os


ROBOT_MODEL = "crx10ia"


def launch_setup(context, *args, **kwargs):

    robot_ip = LaunchConfiguration("robot_ip")
    ros2_control_config = LaunchConfiguration("ros2_control_config")
    use_mock = LaunchConfiguration("use_mock")
    gpio_config_package = LaunchConfiguration("gpio_config_package")
    gpio_config_path = LaunchConfiguration("gpio_config_path")
    motion_control = LaunchConfiguration("motion_control")

    nodes_to_launch = []

    # ------------------------------------------------------------------
    # Physical robot
    # ------------------------------------------------------------------

    include_fanuc_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_hardware_interface"),
                    "launch",
                    "fanuc_physical_control.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_model": ROBOT_MODEL,
            "robot_series": "crx",
            "gpio_config_package": gpio_config_package,
            "gpio_config_path": gpio_config_path,
            "robot_ip": robot_ip,
            "ros2_control_config": ros2_control_config,
            "launch_rviz": "false",
            "use_mock": use_mock,
            "motion_control": motion_control,
        }.items(),
        condition=UnlessCondition(use_mock),
    )

    nodes_to_launch.append(include_fanuc_control)

    # ------------------------------------------------------------------
    # Mock robot
    # ------------------------------------------------------------------

    include_fanuc_mock_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_hardware_interface"),
                    "launch",
                    "fanuc_mock_control.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_model": ROBOT_MODEL,
            "robot_series": "crx",
            "gpio_config_package": gpio_config_package,
            "gpio_config_path": gpio_config_path,
            "ros2_control_config": ros2_control_config,
            "launch_rviz": "false",
        }.items(),
        condition=IfCondition(use_mock),
    )

    nodes_to_launch.append(include_fanuc_mock_control)

    # ------------------------------------------------------------------
    # Robot Description
    # ------------------------------------------------------------------

    description_arguments = {
        "robot_ip": robot_ip.perform(context),
        "use_mock": use_mock.perform(context),
        "gpio_configuration": PathJoinSubstitution(
            [FindPackageShare(gpio_config_package), gpio_config_path]
        ),
    }

    urdf_full_path = os.path.join(
        get_package_share_directory("fanuc_hardware_interface"),
        "robot",
        "crx10ia.urdf.xacro",
    )

    moveit_config = (
        MoveItConfigsBuilder(
            ROBOT_MODEL,
            package_name="fanuc_moveit_config",
        )
        .robot_description(
            file_path=urdf_full_path,
            mappings=description_arguments,
        )
        .robot_description_semantic(
            file_path="srdf/crx10ia.srdf"
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml"
        )
        .robot_description_kinematics(
            file_path="config/kinematics.yaml"
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(
            pipelines=["ompl"]
        )
        .to_moveit_configs()
    )

    # ------------------------------------------------------------------
    # Move Group
    # ------------------------------------------------------------------

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"publish_planning_scene": True},
            {"publish_monitored_planning_scene": True},
        ],
    )

    nodes_to_launch.append(move_group_node)

    # ------------------------------------------------------------------
    # MoveIt Servo Node (REAL-TIME CONTROL LAYER)
    # ------------------------------------------------------------------

    servo_config_file = PathJoinSubstitution([
        FindPackageShare("fanuc_moveit_config"),
        "config",
        "EAS_servo.yaml"
    ])

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        #output="screen",
        parameters=[
            moveit_config.to_dict(),
            servo_config_file,
            {"use_sim_time": False},
        ],
    )


    nodes_to_launch.append(servo_node)
    
    
    shipping_position_node = Node(
        package="test_py",
        executable="ship_pos",
        parameters=[
            moveit_config.to_dict(),
            servo_config_file,
            {"use_sim_time": False},
        ],
    )
    
    nodes_to_launch.append(shipping_position_node)
    # ------------------------------------------------------------------
    # RViz
    # ------------------------------------------------------------------

    rviz_file = PathJoinSubstitution(
        [
            FindPackageShare("fanuc_moveit_config"),
            "rviz",
            "view_robot.rviz",
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="both",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        arguments=[
            "--display-config",
            rviz_file,
        ],
    )

    nodes_to_launch.append(rviz_node)

    return nodes_to_launch


def generate_launch_description():

    declared_arguments = [

        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.100",
            description="The robot IP address.",
        ),

        DeclareLaunchArgument(
            "ros2_control_config",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_hardware_interface"),
                    "config",
                    "ros2_controllers.yaml",
                ]
            ),
            description="ROS2 controller configuration file.",
        ),

        DeclareLaunchArgument(
            "gpio_config_package",
            default_value="fanuc_hardware_interface",
            description="Package containing GPIO configuration.",
        ),

        DeclareLaunchArgument(
            "gpio_config_path",
            default_value="config/example_gpio_config.yaml",
            description="GPIO configuration file.",
        ),

        DeclareLaunchArgument(
            "use_mock",
            default_value="false",
            description="Use mock hardware.",
        ),

        DeclareLaunchArgument(
            "motion_control",
            default_value="1",
            description="Initial motion control state.",
        ),
    ]

    return LaunchDescription(
        declared_arguments
        + [OpaqueFunction(function=launch_setup)]
    )
