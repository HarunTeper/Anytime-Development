"""Single car omnet module launch file"""

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Return launch description"""

    # Package Directories

    threading_type = LaunchConfiguration('threading_type', default='single')

    threading_type_arg = DeclareLaunchArgument(
        'threading_type',
        default_value='single',
        description='Threading type'
    )

    anytime_cmd = Node(
        package='anytime_custom',
        executable='anytime',
        name='anytime',
        output='screen',
        arguments=[threading_type]
    )

    # Launch Description
    launch_description = LaunchDescription()

    # launch_description.add_action(enable_camera_arg)
    launch_description.add_action(threading_type_arg)
    launch_description.add_action(anytime_cmd)
    # launch_description.add_action(realsense_cmd)

    return launch_description
