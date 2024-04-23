"""Single car omnet module launch file"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.launch_context import LaunchContext


def include_launch_description(context: LaunchContext):
    """Include launch description"""

    if context.launch_configurations['threading_type'] == 'single':
        executable = 'component_container'
    elif context.launch_configurations['threading_type'] == 'multi':
        executable = 'component_container_mt'
    else:
        raise ValueError('Invalid threading type')
    
    anytime_cmd = ComposableNodeContainer(
        name='anytime_client_component_container',
        namespace='',
        package='rclcpp_components',
        executable=executable,
        composable_node_descriptions=[
            ComposableNode(
                package='anytime_action',
                plugin='AnytimeActionClient',
                name='anytime_client'
            )
        ]
    )

    cmds = []

    cmds.append(anytime_cmd)

    return cmds

def generate_launch_description():
    """Return launch description"""

    threading_type_arg = DeclareLaunchArgument(
        'threading_type',
        default_value='single',
        description='Threading type'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(threading_type_arg)

    launch_description.add_action(OpaqueFunction(function=include_launch_description))

    return launch_description
