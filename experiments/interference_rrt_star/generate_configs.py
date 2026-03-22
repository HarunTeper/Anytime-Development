#!/usr/bin/env python3
"""
Generate all Interference RRT* experiment configuration files
"""

import os

# Configuration parameters
batch_sizes = [1, 64, 256, 1024, 4096]
modes = ["reactive", "proactive"]
threading = ["single", "multi"]

# Interference timer fixed parameters
TIMER_PERIOD_MS = 100  # 100ms = 10Hz timer frequency
EXECUTION_TIME_MS = 10  # 10ms busy-wait per timer execution

# Depot map coordinates
DEPOT_START_X = 5.0
DEPOT_START_Y = 12.0
DEPOT_GOAL_X = 25.0
DEPOT_GOAL_Y = 2.0

# Base directory
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
config_dir = os.path.join(SCRIPT_DIR, "configs")

# Template for server config (RRT* with map parameters)
server_template = """anytime_server:
  ros__parameters:
    # Anytime algorithm mode
    is_reactive_proactive: "{mode}"  # Options: "reactive", "proactive"

    # Batch processing configuration
    batch_size: {batch_size}  # Number of RRT* iterations to compute per batch

    # Reproducibility
    random_seed: 42  # Seed for reproducible RRT* results

    # Map configuration
    map_yaml_path: "MAPS_DIR/depot.yaml"

    # Start/goal positions (world coordinates in meters)
    start_x: {start_x}
    start_y: {start_y}
    goal_x: {goal_x}
    goal_y: {goal_y}

    # RRT* algorithm parameters
    step_size: 0.5
    goal_threshold: 0.5
    goal_bias: 0.05
    gamma_rrt_star: 0.0    # 0 = auto-compute from map free area
    prune_interval: 1000
    convergence_log_interval: 100
"""

# Template for client config (same for all experiments)
client_template = """anytime_client:
  ros__parameters:
    # Goal timer configuration
    goal_timer_period_ms: 500  # Period in milliseconds for the goal request timer

    # Cancel timeout configuration
    cancel_timeout_period_ms: 200  # Period in milliseconds for the cancel timeout timer
"""

# Template for interference config (same for all experiments)
interference_template = """interference_timer:
  ros__parameters:
    # Timer configuration
    timer_period_ms: {timer_period_ms}  # Period in milliseconds for the interference timer

    # Execution time (busy-wait duration)
    execution_time_ms: {execution_time_ms}  # Busy-wait duration in milliseconds
"""


def main():
    # Create configs directory if it doesn't exist
    os.makedirs(config_dir, exist_ok=True)

    # Generate all combinations
    config_count = 0
    for batch_size in batch_sizes:
        for mode in modes:
            for thread_mode in threading:
                # Create config name
                config_name = f"batch_{batch_size}_{mode}_{thread_mode}"

                # Create server config
                server_content = server_template.format(
                    mode=mode,
                    batch_size=batch_size,
                    start_x=DEPOT_START_X,
                    start_y=DEPOT_START_Y,
                    goal_x=DEPOT_GOAL_X,
                    goal_y=DEPOT_GOAL_Y,
                )

                server_file = os.path.join(
                    config_dir, f"{config_name}_server.yaml")
                with open(server_file, 'w') as f:
                    f.write(server_content)

                # Create client config (same for all)
                client_file = os.path.join(
                    config_dir, f"{config_name}_client.yaml")
                with open(client_file, 'w') as f:
                    f.write(client_template)

                # Create interference config (same for all)
                interference_content = interference_template.format(
                    timer_period_ms=TIMER_PERIOD_MS,
                    execution_time_ms=EXECUTION_TIME_MS
                )
                interference_file = os.path.join(
                    config_dir, f"{config_name}_interference.yaml")
                with open(interference_file, 'w') as f:
                    f.write(interference_content)

                config_count += 1
                print(f"Created config {config_count}: {config_name}")

    print(f"\nTotal configurations created: {config_count}")
    print(f"Configurations saved to: {config_dir}")
    print(f"\nInterference Timer Settings (fixed for all configs):")
    print(f"  - Timer period: {TIMER_PERIOD_MS} ms")
    print(f"  - Execution time: {EXECUTION_TIME_MS} ms")
    print(f"\nNote: MAPS_DIR placeholder in server configs must be replaced")
    print(f"      with the actual installed map path before running experiments.")
    print(f"      The run script handles this automatically.")


if __name__ == "__main__":
    main()
