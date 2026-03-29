# Copyright 2025 Anytime System
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
import os
from ament_index_python.packages import get_package_share_directory


def include_launch_description(context: LaunchContext):
    """Include launch description."""
    is_reactive_proactive = LaunchConfiguration("is_reactive_proactive")
    block_size = LaunchConfiguration("block_size")

    # Get the config file path from context
    config_path = context.launch_configurations.get('config_file', '')

    print("\n" + "="*60)
    print("Monte Carlo Action Server Launch Configuration")
    print("="*60)

    # Determine the threading mode from launch context or config file
    multi_threading_value = context.launch_configurations.get(
        "multi_threading", "")

    # If multi_threading not provided via command line, try to read from YAML config
    if (
        (not multi_threading_value or multi_threading_value == '')
        and config_path and os.path.exists(config_path)
    ):
        import yaml
        try:
            with open(config_path, 'r') as f:
                config_data = yaml.safe_load(f)
                if config_data and 'anytime_server' in config_data:
                    params = config_data['anytime_server'].get(
                        'ros__parameters', {})
                    multi_threading_from_config = params.get(
                        'multi_threading', False)
                    multi_threading_value = (
                        "true" if multi_threading_from_config else "false"
                    )
                    print(f"Loading config from: {config_path}")
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

    if context.launch_configurations.get("multi_threading", "") != "":
        print(
            f"  [Override] multi_threading: {multi_threading_value}"
            f" -> is_single_multi: {is_single_multi} (from command line)")
    else:
        print(
            f"  multi_threading: {multi_threading_value}"
            f" -> is_single_multi: {is_single_multi} (from config file)")

    # Prepare parameters - either from config file or command line arguments
    if config_path and os.path.exists(config_path):
        # Use config file
        parameters = [config_path]
        # Override with command line arguments if provided
        overrides = {}
        reactive_proactive_value = context.launch_configurations.get(
            'is_reactive_proactive', '')
        block_size_value = context.launch_configurations.get('block_size', '')

        if reactive_proactive_value and reactive_proactive_value != '':
            overrides['is_reactive_proactive'] = reactive_proactive_value
            print(
                f"  [Override] is_reactive_proactive:"
                f" {reactive_proactive_value} (from command line)")
        if block_size_value and block_size_value != '':
            overrides['block_size'] = int(block_size_value)
            print(
                f"  [Override] block_size: {block_size_value} (from command line)")
        overrides['is_single_multi'] = is_single_multi
        if overrides:
            parameters.append(overrides)
    else:
        print("Using command line arguments (no config file)")
        # Use command line arguments
        parameters = [{
            "is_reactive_proactive": is_reactive_proactive,
            "block_size": block_size,
            "is_single_multi": is_single_multi,
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
        package="anytime_monte_carlo",
        executable="anytime_mc_server",
        name="anytime_server",
        parameters=parameters,
        arguments=arguments
    )

    cmds = []
    cmds.append(anytime_cmd)

    return cmds


def generate_launch_description():
    """Return launch description."""
    # Get the package directory for anytime_monte_carlo
    try:
        package_dir = get_package_share_directory('anytime_monte_carlo')
        default_config_file = os.path.join(
            package_dir, 'config', 'server_params.yaml')
    except Exception as e:
        print(f"Warning: Could not get package directory: {e}")
        default_config_file = ""

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to the server configuration YAML file'
    )

    threading_type_arg = DeclareLaunchArgument(
        "multi_threading",
        default_value="",
        description="Threading type (overrides config file)"
    )

    anytime_reactive_proactive_arg = DeclareLaunchArgument(
        "is_reactive_proactive",
        default_value="",
        description="Anytime reactive (overrides config file)"
    )

    block_size_arg = DeclareLaunchArgument(
        "block_size",
        default_value="",
        description="Block size for compute iterations (overrides config)"
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="",
        description="Logging level (debug, info, warn, error, fatal)"
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(config_file_arg)
    launch_description.add_action(threading_type_arg)
    launch_description.add_action(anytime_reactive_proactive_arg)
    launch_description.add_action(block_size_arg)
    launch_description.add_action(log_level_arg)

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
