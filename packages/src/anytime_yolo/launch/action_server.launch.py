from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
import os
from ament_index_python.packages import get_package_share_directory


def include_launch_description(context: LaunchContext):
    """Include launch description"""
    is_reactive_proactive = LaunchConfiguration("is_reactive_proactive")
    batch_size = LaunchConfiguration("batch_size")
    
    # Determine the threading mode from launch context
    if context.launch_configurations["multi_threading"].lower() == "false":
        is_single_multi = "single"
    elif context.launch_configurations["multi_threading"].lower() == "true":
        is_single_multi = "multi"
    else:
        raise ValueError("Invalid threading type")

    anytime_cmd = Node(
        package="anytime_yolo",
        executable="anytime_action_server",
        name="anytime_server",
        parameters=[{
            "is_reactive_proactive": is_reactive_proactive,
            "batch_size": batch_size,
            "is_single_multi": is_single_multi,
            "weights_path": context.launch_configurations["weights_path"]
        }],
        arguments=["--is_single_multi", is_single_multi]
    )

    cmds = []

    cmds.append(anytime_cmd)

    return cmds


def generate_launch_description():
    """Return launch description"""

    # Get the package directory for anytime_yolo
    try:
        package_dir = get_package_share_directory('anytime_yolo')
        default_weights_path = os.path.join(package_dir, "weights_32")
    except Exception as e:
        print(f"Warning: Could not get package directory: {e}")
        default_weights_path = ""

    threading_type_arg = DeclareLaunchArgument(
        "multi_threading", default_value="False", description="Threading type"
    )

    anytime_reactive_proactive_arg = DeclareLaunchArgument(
        "is_reactive_proactive", default_value="reactive", description="Anytime reactive"
    )

    batch_size_arg = DeclareLaunchArgument(
        "batch_size", default_value="2", description="Batch size for compute iterations"
    )

    weights_path_arg = DeclareLaunchArgument(
        "weights_path", default_value=default_weights_path,
        description="Path to the anytimeyolo weights directory"
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(threading_type_arg)
    launch_description.add_action(anytime_reactive_proactive_arg)
    launch_description.add_action(batch_size_arg)
    launch_description.add_action(weights_path_arg)

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
