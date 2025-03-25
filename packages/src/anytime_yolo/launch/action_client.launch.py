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

    cancel_timeout_period = LaunchConfiguration('cancel_timeout_period_ms')
    result_filename = LaunchConfiguration('result_filename')
    image_topic = LaunchConfiguration('image_topic')

    anytime_cmd = ComposableNodeContainer(
        name='anytime_client_component_container',
        namespace='',
        package='rclcpp_components',
        executable=executable,
        composable_node_descriptions=[
            ComposableNode(
                package='anytime_yolo',
                plugin='AnytimeActionClient',
                name='anytime_client',
                parameters=[{
                    'cancel_timeout_period_ms': cancel_timeout_period,
                    'result_filename': result_filename,
                    'image_topic': image_topic
                }]
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

    cancel_timeout_period_arg = DeclareLaunchArgument(
        'cancel_timeout_period_ms',
        default_value='20',
        description='Period in milliseconds for the cancel timeout timer'
    )

    result_filename_arg = DeclareLaunchArgument(
        'result_filename',
        default_value='anytime_results',
        description='Filename for storing results (without extension)'
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='video_frames',
        description='Topic name for the input image'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(threading_type_arg)
    launch_description.add_action(cancel_timeout_period_arg)
    launch_description.add_action(result_filename_arg)
    launch_description.add_action(image_topic_arg)

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
