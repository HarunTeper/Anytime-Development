"""Launch file for video publisher only - used in experiments."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Return launch description with video publisher only."""

    # Get default image path
    video_publisher_pkg = get_package_share_directory('video_publisher')
    default_image_path = os.path.join(video_publisher_pkg, 'images')

    # Declare launch arguments
    image_path_arg = DeclareLaunchArgument(
        'image_path',
        default_value=default_image_path,
        description='Path to the image directory for video publisher'
    )

    # Video publisher node
    video_publisher = Node(
        package='video_publisher',
        executable='video_publisher.py',
        name='video_publisher',
        parameters=[{
            'image_path': LaunchConfiguration('image_path')
        }],
        output='screen'
    )

    return LaunchDescription([
        image_path_arg,
        video_publisher
    ])
