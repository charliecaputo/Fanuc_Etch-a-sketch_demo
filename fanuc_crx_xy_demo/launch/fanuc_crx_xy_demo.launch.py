from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('fanuc_crx_xy_demo'),
        'config',
        'demo_params.yaml'
    ])

    return LaunchDescription([
        Node(
            package='fanuc_crx_xy_demo',
            executable='dual_as5600_mux_node',
            name='dual_as5600_mux_node',
            output='screen',
            parameters=[params]
        ),
        Node(
            package='fanuc_crx_xy_demo',
            executable='xy_target_node',
            name='xy_target_node',
            output='screen',
            parameters=[params]
        ),
        Node(
            package='fanuc_crx_xy_demo',
            executable='fanuc_moveit_target_bridge_example',
            name='fanuc_moveit_target_bridge_example',
            output='screen'
        ),
    ])
