from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='color_lidar',
            executable='trans_command_node',
            name='trans_command_node',
            output='screen'
        ),
        Node(
            package='color_lidar',
            executable='move_control_node',
            name='move_control_node',
            output='screen'
        ),
        Node(
            package='color_lidar',
            executable='lidar_node',
            name='lidar_node',
            output='screen'
        ),
    ])
