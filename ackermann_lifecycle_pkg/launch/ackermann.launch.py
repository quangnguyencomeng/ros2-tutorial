from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ackermann_lifecycle_pkg',
            executable='ackermann_kinematics_node',
            name='ackermann_kinematics_node',
            parameters=[{
                'wheelbase': 1.5,
                'track_width_front': 1.0,
            }],
            output='screen',
        )
    ])