from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_context import LaunchContext


def include_launch_description(context: LaunchContext):
    """Include launch description"""

    result_filename = LaunchConfiguration('result_filename')
    image_topic = LaunchConfiguration('image_topic')
    cancel_after_layers = LaunchConfiguration('cancel_after_layers')

    anytime_cmd = Node(
        package='anytime_yolo',
        executable='anytime_yolo_client',
        name='anytime_client',
        parameters=[{
            'result_filename': result_filename,
            'image_topic': image_topic,
            'cancel_after_layers': cancel_after_layers
        }]
    )

    cmds = []
    cmds.append(anytime_cmd)

    return cmds


def generate_launch_description():
    """Return launch description"""

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

    cancel_after_layers_arg = DeclareLaunchArgument(
        'cancel_after_layers',
        default_value='15',
        description='Number of processed layers before triggering cancellation'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(result_filename_arg)
    launch_description.add_action(image_topic_arg)
    launch_description.add_action(cancel_after_layers_arg)

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
