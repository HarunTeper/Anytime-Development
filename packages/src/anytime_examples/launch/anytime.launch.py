"""Single car omnet module launch file"""

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    """Return launch description"""

    # Package Directories
    pkg_dir = get_package_share_directory('anytime_examples')

    threading_type = LaunchConfiguration('threading_type', default='single')

    threading_type_arg = DeclareLaunchArgument(
        'threading_type',
        default_value='single',
        description='Threading type'
    )

    yolo_cmd = Node(
        package='anytime_examples',
        executable='anytime',
        name='anytime',
        output='screen',
        arguments=[threading_type]
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(threading_type_arg)
    launch_description.add_action(yolo_cmd)

    return launch_description
