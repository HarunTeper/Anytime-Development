from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test_action',
            executable='test_action_server',
            name='test_action_server',
            output='screen',
            arguments=['--ros-args', '--log-level', 'debug']
        ),
    ])
