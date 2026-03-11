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
    config_path = context.launch_configurations.get('config_file', '')

    print("\n" + "="*60)
    print("RRT* Action Client Launch Configuration")
    print("="*60)

    if config_path and os.path.exists(config_path):
        parameters = [config_path]
        overrides = {}
        goal_timer_value = context.launch_configurations.get(
            'goal_timer_period_ms', '')
        cancel_timeout_value = context.launch_configurations.get(
            'cancel_timeout_period_ms', '')

        if goal_timer_value and goal_timer_value != '':
            overrides['goal_timer_period_ms'] = int(goal_timer_value)
            print(f"  [Override] goal_timer_period_ms: {goal_timer_value}")
        if cancel_timeout_value and cancel_timeout_value != '':
            overrides['cancel_timeout_period_ms'] = int(cancel_timeout_value)
            print(f"  [Override] cancel_timeout_period_ms: {cancel_timeout_value}")
        if overrides:
            parameters.append(overrides)
        print(f"  Loading config from: {config_path}")
    else:
        print("Using command line arguments (no config file)")
        parameters = [{
            "goal_timer_period_ms": LaunchConfiguration("goal_timer_period_ms"),
            "cancel_timeout_period_ms": LaunchConfiguration("cancel_timeout_period_ms"),
        }]

    logger = context.launch_configurations.get('log_level', '')
    if not logger or logger == '':
        logger = 'info'
    print(f"  log_level: {logger}")
    print("="*60 + "\n")

    anytime_cmd = Node(
        package="anytime_rrt_star",
        executable="anytime_rrt_client",
        name="anytime_client",
        parameters=parameters,
        arguments=["--ros-args", "--log-level", logger]
    )

    return [anytime_cmd]


def generate_launch_description():
    """Return launch description."""
    try:
        package_dir = get_package_share_directory('anytime_rrt_star')
        default_config_file = os.path.join(
            package_dir, 'config', 'client_params.yaml')
    except Exception as e:
        print(f"Warning: Could not get package directory: {e}")
        default_config_file = ""

    launch_description = LaunchDescription()

    launch_description.add_action(DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to the client configuration YAML file'
    ))

    launch_description.add_action(DeclareLaunchArgument(
        "goal_timer_period_ms",
        default_value="",
        description="Goal timer period in ms (overrides config file)"
    ))

    launch_description.add_action(DeclareLaunchArgument(
        "cancel_timeout_period_ms",
        default_value="",
        description="Cancel timeout in ms (overrides config file)"
    ))

    launch_description.add_action(DeclareLaunchArgument(
        "log_level",
        default_value="",
        description="Logging level (debug, info, warn, error, fatal)"
    ))

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
