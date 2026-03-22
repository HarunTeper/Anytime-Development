from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
import os
from ament_index_python.packages import get_package_share_directory


def include_launch_description(context: LaunchContext):
    """Include launch description"""

    config_file = LaunchConfiguration("config_file")
    is_reactive_proactive = LaunchConfiguration("is_reactive_proactive")
    block_size = LaunchConfiguration("block_size")
    is_passive_cooperative = LaunchConfiguration("is_passive_cooperative")
    is_sync_async = LaunchConfiguration("is_sync_async")
    weights_path = LaunchConfiguration("weights_path")
    log_level = LaunchConfiguration("log_level")

    # Get the config file path from context
    config_path = context.launch_configurations.get('config_file', '')

    print("\n" + "="*60)
    print("YOLO Action Server Launch Configuration")
    print("="*60)

    # Determine the threading mode from launch context or config file
    multi_threading_value = context.launch_configurations.get(
        "multi_threading", "")

    # If multi_threading not provided via command line, try to read from YAML config
    if (not multi_threading_value or multi_threading_value == '') and config_path and os.path.exists(config_path):
        import yaml
        try:
            with open(config_path, 'r') as f:
                config_data = yaml.safe_load(f)
                if config_data and 'anytime_server' in config_data:
                    params = config_data['anytime_server'].get(
                        'ros__parameters', {})
                    multi_threading_from_config = params.get(
                        'multi_threading', False)
                    multi_threading_value = "true" if multi_threading_from_config else "false"
        except Exception as e:
            print(
                f"Warning: Could not read multi_threading from config file: {e}")
            multi_threading_value = "false"
    elif not multi_threading_value or multi_threading_value == '':
        multi_threading_value = "false"

    if multi_threading_value.lower() == "false":
        is_single_multi = "single"
    elif multi_threading_value.lower() == "true":
        is_single_multi = "multi"
    else:
        raise ValueError("Invalid threading type")

    # Get weights path from context or use default
    weights_path_value = context.launch_configurations.get("weights_path", "")
    if not weights_path_value:
        try:
            package_dir = get_package_share_directory('anytime_yolo')
            weights_path_value = os.path.join(package_dir, "weights_32")
        except Exception as e:
            print(f"Warning: Could not get package directory: {e}")
            weights_path_value = ""

    # Prepare parameters - either from config file or command line arguments
    if config_path and os.path.exists(config_path):
        print(f"Loading config from: {config_path}")

        # Read config to show parameters
        import yaml
        try:
            with open(config_path, 'r') as f:
                config_data = yaml.safe_load(f)
                if config_data and 'anytime_server' in config_data:
                    params = config_data['anytime_server'].get(
                        'ros__parameters', {})
                    print(
                        f"  is_reactive_proactive: {params.get('is_reactive_proactive', 'N/A')} (from config file)")
                    print(
                        f"  block_size: {params.get('block_size', 'N/A')} (from config file)")
                    print(
                        f"  multi_threading: {params.get('multi_threading', 'N/A')} (from config file)")
                    print(
                        f"  is_passive_cooperative: {params.get('is_passive_cooperative', 'N/A')} (from config file)")
                    print(
                        f"  is_sync_async: {params.get('is_sync_async', 'N/A')} (from config file)")
                    print(
                        f"  weights_path: {params.get('weights_path', 'N/A')} (from config file)")
        except Exception as e:
            print(f"Warning: Could not read parameters from config file: {e}")

        # Use config file
        parameters = [config_path]
        # Override with command line arguments if provided
        overrides = {}
        reactive_proactive_value = context.launch_configurations.get(
            'is_reactive_proactive', '')
        block_size_value = context.launch_configurations.get('block_size', '')
        sync_async_value = context.launch_configurations.get(
            'is_sync_async', '')

        if reactive_proactive_value and reactive_proactive_value != '':
            overrides['is_reactive_proactive'] = reactive_proactive_value
            print(
                f"  [Override] is_reactive_proactive: {reactive_proactive_value} (from command line)")
        if block_size_value and block_size_value != '':
            overrides['block_size'] = int(block_size_value)
            print(
                f"  [Override] block_size: {block_size_value} (from command line)")

        multi_threading_cmd = context.launch_configurations.get(
            "multi_threading", "")
        if multi_threading_cmd and multi_threading_cmd != '':
            print(
                f"  [Override] multi_threading: {multi_threading_cmd} (from command line)")

        overrides['is_single_multi'] = is_single_multi
        print(
            f"  is_single_multi: {is_single_multi} (derived from multi_threading)")

        if sync_async_value and sync_async_value != '':
            overrides['is_sync_async'] = sync_async_value
            print(
                f"  [Override] is_sync_async: {sync_async_value} (from command line)")
        if weights_path_value:
            weights_path_cmd = context.launch_configurations.get(
                "weights_path", "")
            if weights_path_cmd and weights_path_cmd != '':
                overrides['weights_path'] = weights_path_value
                print(
                    f"  [Override] weights_path: {weights_path_value} (from command line)")
            else:
                overrides['weights_path'] = weights_path_value
                print(f"  weights_path: {weights_path_value} (default)")
        if overrides:
            parameters.append(overrides)
    else:
        print("Using command line arguments (no config file)")
        # Use command line arguments
        reactive_proactive_value = context.launch_configurations.get(
            'is_reactive_proactive', '')
        block_size_value = context.launch_configurations.get('block_size', '')
        sync_async_value = context.launch_configurations.get(
            'is_sync_async', '')

        print(
            f"  is_reactive_proactive: {reactive_proactive_value} (from command line)")
        print(f"  block_size: {block_size_value} (from command line)")
        print(
            f"  multi_threading: {multi_threading_value} (from command line)")
        print(
            f"  is_single_multi: {is_single_multi} (derived from multi_threading)")
        print(f"  is_sync_async: {sync_async_value} (from command line)")
        print(
            f"  weights_path: {weights_path_value} (from command line or default)")

        parameters = [{
            "is_reactive_proactive": is_reactive_proactive,
            "block_size": block_size,
            "is_single_multi": is_single_multi,
            "weights_path": weights_path_value,
            "is_sync_async": is_sync_async
        }]

    # Get logger level from config or command line
    logger = context.launch_configurations.get('log_level', '')

    # If log_level not provided via command line, try to read from YAML config
    if (not logger or logger == '') and config_path and os.path.exists(config_path):
        import yaml
        try:
            with open(config_path, 'r') as f:
                config_data = yaml.safe_load(f)
                if config_data and 'anytime_server' in config_data:
                    params = config_data['anytime_server'].get(
                        'ros__parameters', {})
                    logger = params.get('log_level', 'info')
                    print(f"  log_level: {logger} (from config file)")
        except Exception as e:
            print(f"Warning: Could not read log_level from config file: {e}")
            logger = 'info'
            print(f"  log_level: {logger} (default)")
    elif logger and logger != '':
        print(f"  [Override] log_level: {logger} (from command line)")
    else:
        logger = 'info'
        print(f"  log_level: {logger} (default)")

    print("="*60 + "\n")

    # Build arguments list
    arguments = ["--is_single_multi", is_single_multi,
                 "--ros-args", "--log-level", logger]

    anytime_cmd = Node(
        package="anytime_yolo",
        executable="anytime_yolo_server",
        name="anytime_server",
        parameters=parameters,
        arguments=arguments,
        # output='screen',
    )

    cmds = []

    cmds.append(anytime_cmd)

    return cmds


def generate_launch_description():
    """Return launch description"""

    # Get the package directory for anytime_yolo
    try:
        package_dir = get_package_share_directory('anytime_yolo')
        default_config_file = os.path.join(
            package_dir, 'config', 'server_params.yaml')
        default_weights_path = os.path.join(package_dir, "weights_32")
    except Exception as e:
        print(f"Warning: Could not get package directory: {e}")
        default_config_file = ""
        default_weights_path = ""

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to the server configuration YAML file'
    )

    threading_type_arg = DeclareLaunchArgument(
        "multi_threading", default_value="", description="Threading type (overrides config file)"
    )

    anytime_reactive_proactive_arg = DeclareLaunchArgument(
        "is_reactive_proactive", default_value="", description="Anytime reactive (overrides config file)"
    )

    block_size_arg = DeclareLaunchArgument(
        "block_size", default_value="", description="Block size for compute iterations (overrides config file)"
    )

    weights_path_arg = DeclareLaunchArgument(
        "weights_path", default_value="",
        description="Path to the anytimeyolo weights directory (overrides config file)"
    )

    sync_async_arg = DeclareLaunchArgument(
        "is_sync_async", default_value="",
        description="Synchronous or asynchronous mode (overrides config file)"
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="", description="Logging level (debug, info, warn, error, fatal)"
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(config_file_arg)
    launch_description.add_action(threading_type_arg)
    launch_description.add_action(anytime_reactive_proactive_arg)
    launch_description.add_action(block_size_arg)
    launch_description.add_action(weights_path_arg)
    launch_description.add_action(sync_async_arg)
    launch_description.add_action(log_level_arg)

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
