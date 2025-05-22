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
    debug = LaunchConfiguration('debug')
    cancel_layer_score = LaunchConfiguration('cancel_layer_score')

    # Set logger level based on debug argument
    logger = "debug" if context.launch_configurations.get(
        "debug", "false").lower() == "true" else "info"

    anytime_cmd = Node(
        package='anytime_yolo',
        executable='anytime_yolo_client',
        name='anytime_client',
        parameters=[{
            'result_filename': result_filename,
            'image_topic': image_topic,
            'cancel_after_layers': cancel_after_layers,
            'cancel_layer_score': cancel_layer_score
        }],
        arguments=['--ros-args', '--log-level', logger],
        # output='screen',
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
        default_value='12',
        description='Number of processed layers before triggering cancellation'
    )

    debug_arg = DeclareLaunchArgument(
        'debug', default_value='false', description='Enable debug logging'
    )

    cancel_layer_score_arg = DeclareLaunchArgument(
        'cancel_layer_score',
        default_value='false',
        description='Enable cancellation based on high score for id 9 in feedback'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(result_filename_arg)
    launch_description.add_action(image_topic_arg)
    launch_description.add_action(cancel_after_layers_arg)
    launch_description.add_action(debug_arg)
    launch_description.add_action(cancel_layer_score_arg)

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
