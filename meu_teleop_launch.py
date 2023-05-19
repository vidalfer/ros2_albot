from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_control',
            executable='teleop_node',
            output='screen',
            parameters=[{'linear_speed': 0.5, 'angular_speed': 0.5, 'acceleration_factor': 2.0}],
        ),
    ])
