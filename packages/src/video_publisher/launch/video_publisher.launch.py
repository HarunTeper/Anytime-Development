from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_path = get_package_share_directory('video_publisher')
    default_image_path = os.path.join(pkg_path, 'images', 'sample.png')

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_path',
            default_value=default_image_path,
            description='Path to the image file'
        ),

        Node(
            package='video_publisher',
            executable='video_publisher.py',
            name='video_publisher',
            parameters=[{
                'image_path': LaunchConfiguration('image_path')
            }],
            output='screen'
        )
    ])
