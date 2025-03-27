from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_context import LaunchContext


def include_launch_description(context: LaunchContext):
    """Include launch description"""

    cancel_timeout_period = LaunchConfiguration('cancel_timeout_period_ms')
    result_filename = LaunchConfiguration('result_filename')
    image_topic = LaunchConfiguration('image_topic')

    anytime_cmd = Node(
        package='anytime_yolo',
        executable='anytime_action_client',
        name='anytime_client',
        parameters=[{
            'cancel_timeout_period_ms': cancel_timeout_period,
            'result_filename': result_filename,
            'image_topic': image_topic
        }]
    )

    cmds = []
    cmds.append(anytime_cmd)

    return cmds


def generate_launch_description():
    """Return launch description"""

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

    launch_description.add_action(cancel_timeout_period_arg)
    launch_description.add_action(result_filename_arg)
    launch_description.add_action(image_topic_arg)

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
