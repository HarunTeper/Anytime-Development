"""Launch file for Interference experiment with Monte Carlo server, client, and interference timer components."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Setup function to print executor type and return container(s)."""

    use_multi_threaded = context.launch_configurations.get(
        'use_multi_threaded', 'true')
    container_name = context.launch_configurations.get(
        'container_name', 'interference_experiment_container')

    executor_type = "MULTI-THREADED" if use_multi_threaded == 'true' else "SINGLE-THREADED"
    executable_name = "component_container_mt" if use_multi_threaded == 'true' else "component_container"

    print("\n" + "="*80)
    print("Interference Experiment Launch")
    print("="*80)
    print(f"Container Name: {container_name}")
    print(f"Executor Type:  {executor_type}")
    print(f"Executable:     {executable_name}")
    print("="*80 + "\n")

    # Get the experiments package directory
    experiments_dir = get_package_share_directory('experiments')

    # Default config files from experiments package
    default_client_config = os.path.join(
        experiments_dir, 'config', 'interference', 'default_client.yaml')
    default_server_config = os.path.join(
        experiments_dir, 'config', 'interference', 'default_server.yaml')
    default_interference_config = os.path.join(
        experiments_dir, 'config', 'interference', 'default_interference.yaml')

    client_config = context.launch_configurations.get(
        'client_config', default_client_config)
    server_config = context.launch_configurations.get(
        'server_config', default_server_config)
    interference_config = context.launch_configurations.get(
        'interference_config', default_interference_config)
    log_level = context.launch_configurations.get('log_level', 'info')

    # Get interference timer parameters
    timer_period_ms = context.launch_configurations.get(
        'timer_period_ms', '100')
    execution_time_ms = context.launch_configurations.get(
        'execution_time_ms', '10')

    if use_multi_threaded == 'true':
        # Multi-threaded: Client and Server (with interference) in the same container
        print("Configuration: Client and Server (with Interference) in SAME container (multi-threaded)")
        print("="*80 + "\n")

        container = ComposableNodeContainer(
            name=container_name,
            namespace='',
            package='rclcpp_components',
            executable=executable_name,
            composable_node_descriptions=[
                ComposableNode(
                    package='anytime_monte_carlo',
                    plugin='AnytimeActionServer',
                    name='anytime_server',
                    parameters=[server_config],
                ),
                ComposableNode(
                    package='interference',
                    plugin='interference::InterferenceTimerNode',
                    name='interference_timer',
                    parameters=[{
                        'timer_period_ms': int(timer_period_ms),
                        'execution_time_ms': int(execution_time_ms),
                    }]
                ),
                ComposableNode(
                    package='anytime_monte_carlo',
                    plugin='AnytimeActionClient',
                    name='anytime_client',
                    parameters=[client_config],
                ),
            ],
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
        )
        return [container]
    else:
        # Single-threaded: Separate containers - server with interference, and client
        print("Configuration: Server (with Interference) and Client in SEPARATE containers (single-threaded)")
        print("="*80 + "\n")

        server_container = ComposableNodeContainer(
            name=f'{container_name}_server',
            namespace='',
            package='rclcpp_components',
            executable=executable_name,
            composable_node_descriptions=[
                ComposableNode(
                    package='anytime_monte_carlo',
                    plugin='AnytimeActionServer',
                    name='anytime_server',
                    parameters=[server_config],
                ),
                ComposableNode(
                    package='interference',
                    plugin='interference::InterferenceTimerNode',
                    name='interference_timer',
                    parameters=[{
                        'timer_period_ms': int(timer_period_ms),
                        'execution_time_ms': int(execution_time_ms),
                    }]
                ),
            ],
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
        )

        client_container = ComposableNodeContainer(
            name=f'{container_name}_client',
            namespace='',
            package='rclcpp_components',
            executable=executable_name,
            composable_node_descriptions=[
                ComposableNode(
                    package='anytime_monte_carlo',
                    plugin='AnytimeActionClient',
                    name='anytime_client',
                    parameters=[client_config],
                ),
            ],
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
        )

        return [server_container, client_container]


def generate_launch_description():
    """Return launch description with component container."""

    # Get the experiments package directory
    experiments_dir = get_package_share_directory('experiments')

    # Default config files from experiments package
    default_client_config = os.path.join(
        experiments_dir, 'config', 'interference', 'default_client.yaml')
    default_server_config = os.path.join(
        experiments_dir, 'config', 'interference', 'default_server.yaml')
    default_interference_config = os.path.join(
        experiments_dir, 'config', 'interference', 'default_interference.yaml')

    # Declare launch arguments
    client_config_arg = DeclareLaunchArgument(
        'client_config',
        default_value=default_client_config,
        description='Path to client configuration YAML file'
    )

    server_config_arg = DeclareLaunchArgument(
        'server_config',
        default_value=default_server_config,
        description='Path to server configuration YAML file'
    )

    interference_config_arg = DeclareLaunchArgument(
        'interference_config',
        default_value=default_interference_config,
        description='Path to interference timer configuration YAML file'
    )

    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='interference_experiment_container',
        description='Name of the component container'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )

    use_multi_threaded_arg = DeclareLaunchArgument(
        'use_multi_threaded',
        default_value='true',
        description='Use multi-threaded executor (true) or single-threaded (false)'
    )

    timer_period_arg = DeclareLaunchArgument(
        'timer_period_ms',
        default_value='100',
        description='Timer period in milliseconds for interference timer'
    )

    execution_time_arg = DeclareLaunchArgument(
        'execution_time_ms',
        default_value='10',
        description='Execution time (busy-wait duration) in milliseconds for interference timer'
    )

    return LaunchDescription([
        client_config_arg,
        server_config_arg,
        interference_config_arg,
        container_name_arg,
        log_level_arg,
        use_multi_threaded_arg,
        timer_period_arg,
        execution_time_arg,
        OpaqueFunction(function=launch_setup)
    ])
