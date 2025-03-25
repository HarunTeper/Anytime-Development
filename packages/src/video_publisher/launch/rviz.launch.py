import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the path to the RViz configuration file
    package_name = 'video_publisher'
    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'config.rviz')

    print("rviz_config_path: ", rviz_config_path)

    # Launch RViz with the specified configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen')

    return LaunchDescription([rviz_node])
