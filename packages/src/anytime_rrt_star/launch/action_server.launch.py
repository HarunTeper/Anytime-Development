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
    batch_size = LaunchConfiguration("batch_size")

    config_path = context.launch_configurations.get('config_file', '')

    print("\n" + "="*60)
    print("RRT* Action Server Launch Configuration")
    print("="*60)

    multi_threading_value = context.launch_configurations.get(
        "multi_threading", "")

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

    if config_path and os.path.exists(config_path):
        parameters = [config_path]
        overrides = {}
        reactive_proactive_value = context.launch_configurations.get(
            'is_reactive_proactive', '')
        batch_size_value = context.launch_configurations.get('batch_size', '')

        if reactive_proactive_value and reactive_proactive_value != '':
            overrides['is_reactive_proactive'] = reactive_proactive_value
            print(
                f"  [Override] is_reactive_proactive:"
                f" {reactive_proactive_value} (from command line)")
        if batch_size_value and batch_size_value != '':
            overrides['batch_size'] = int(batch_size_value)
            print(
                f"  [Override] batch_size: {batch_size_value} (from command line)")
        overrides['is_single_multi'] = is_single_multi
        if overrides:
            parameters.append(overrides)
    else:
        print("Using command line arguments (no config file)")
        parameters = [{
            "is_reactive_proactive": is_reactive_proactive,
            "batch_size": batch_size,
            "is_single_multi": is_single_multi,
        }]

    logger = context.launch_configurations.get('log_level', '')

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

    arguments = ["--is_single_multi", is_single_multi,
                 "--ros-args", "--log-level", logger]

    anytime_cmd = Node(
        package="anytime_rrt_star",
        executable="anytime_rrt_server",
        name="anytime_server",
        parameters=parameters,
        arguments=arguments
    )

    return [anytime_cmd]


def generate_launch_description():
    """Return launch description."""
    try:
        package_dir = get_package_share_directory('anytime_rrt_star')
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

    batch_size_arg = DeclareLaunchArgument(
        "batch_size",
        default_value="",
        description="Batch size for compute iterations (overrides config)"
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="",
        description="Logging level (debug, info, warn, error, fatal)"
    )

    launch_description = LaunchDescription()

    launch_description.add_action(config_file_arg)
    launch_description.add_action(threading_type_arg)
    launch_description.add_action(anytime_reactive_proactive_arg)
    launch_description.add_action(batch_size_arg)
    launch_description.add_action(log_level_arg)

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
