from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pigpio',
            executable='servo',
            output='screen'
        ),
        Node(
            package='pigpio',
            executable='controller',
        )
    ])
