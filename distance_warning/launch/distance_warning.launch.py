from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    threshold_arg = DeclareLaunchArgument(
        'threshold',
        default_value='0.5',
        description='Distance warning threshold in meters'
    )
    threshold = LaunchConfiguration('threshold')

    return LaunchDescription([
        threshold_arg,
        Node(
            package='distance_warning',
            executable='distance_publisher',
            name='distance_publisher'
        ),
        Node(
            package='distance_warning',
            executable='distance_listener',
            name='distance_listener',
            parameters=[{'threshold': threshold}]
        ),
        Node(
            package='distance_warning',
            executable='set_threshold_service',
            name='set_threshold_service',
            parameters=[{'threshold': threshold}]
        ),
        Node(
            package='distance_warning',
            executable='distance_action_server',
            name='distance_action_server',
            parameters=[{'threshold': threshold}]
        ),
    ])

# ros2 launch distance_warning distance_warning.launch.py