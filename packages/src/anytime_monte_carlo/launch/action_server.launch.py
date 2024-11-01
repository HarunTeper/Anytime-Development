"""Single car omnet module launch file"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.launch_context import LaunchContext


def include_launch_description(context: LaunchContext):
    """Include launch description"""

    if context.launch_configurations["multi_threading"] == "False":
        executable = "component_container"
    elif context.launch_configurations["multi_threading"] == "True":
        executable = "component_container_mt"
    else:
        raise ValueError("Invalid threading type")

    anytime_cmd = ComposableNodeContainer(
        name="anytime_server_component_container",
        namespace="",
        package="rclcpp_components",
        executable=executable,
        parameters=[{"thread_num": 2}],
        composable_node_descriptions=[
            ComposableNode(
                package="anytime_monte_carlo",
                plugin="AnytimeActionServer",
                name="anytime_server",
                parameters=[
                    {"anytime_active": LaunchConfiguration("anytime_active")},
                    {"separate_thread": LaunchConfiguration("separate_thread")},
                    {"multi_threading": LaunchConfiguration("multi_threading")},
                ],
            )
        ],
    )

    cmds = []

    cmds.append(anytime_cmd)

    return cmds


def generate_launch_description():
    """Return launch description"""

    threading_type_arg = DeclareLaunchArgument(
        "multi_threading", default_value="True", description="Threading type"
    )

    anytime_active_arg = DeclareLaunchArgument(
        "anytime_active", default_value="False", description="Anytime active"
    )

    separate_thread_arg = DeclareLaunchArgument(
        "separate_thread", default_value="False", description="Separate thread"
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(threading_type_arg)
    launch_description.add_action(anytime_active_arg)
    launch_description.add_action(separate_thread_arg)

    launch_description.add_action(OpaqueFunction(function=include_launch_description))

    return launch_description
