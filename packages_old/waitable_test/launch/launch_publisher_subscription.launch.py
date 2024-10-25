"""ROS2 Launch file"""

from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    """Returns launch description"""

    # Nodes
    publisher_node = Node(
        package='waitable_test',
        executable='basic_publisher',
        name='basic_publisher',
        output='screen'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(publisher_node)

    return launch_description
