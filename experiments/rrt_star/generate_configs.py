#!/usr/bin/env python3
"""
Generate all RRT* experiment configuration files
"""

import os

# Configuration parameters
batch_sizes = [1, 16, 64, 256, 1024, 4096, 16384]
modes = ["reactive", "proactive"]
threading = ["single", "multi"]
maps = {
    "depot": {
        "start_x": 5.0, "start_y": 12.0,
        "goal_x": 25.0, "goal_y": 2.0,
    },
    "warehouse": {
        "start_x": -12.5, "start_y": -20.0,
        "goal_x": 12.5, "goal_y": 20.0,
    },
}

# Base directory
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
config_dir = os.path.join(SCRIPT_DIR, "configs")

# Template for server config
server_template = """anytime_server:
  ros__parameters:
    # Anytime algorithm mode
    is_reactive_proactive: "{mode}"  # Options: "reactive", "proactive"

    # Threading configuration
    multi_threading: {multi_threading}  # Enable/disable multi-threading

    # Batch processing configuration
    batch_size: {batch_size}  # Number of iterations to compute per batch

    # Reproducibility
    random_seed: 42  # Seed for reproducible RRT* results

    # Logging configuration
    log_level: "info"  # Options: "debug", "info", "warn", "error", "fatal"

    # Map configuration
    map_yaml_path: "{map_yaml_path}"

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

    # Logging configuration
    log_level: "info"  # Options: "debug", "info", "warn", "error", "fatal"
"""


def main():
    # Create configs directory if it doesn't exist
    os.makedirs(config_dir, exist_ok=True)

    # Generate all combinations
    config_count = 0
    for batch_size in batch_sizes:
        for mode in modes:
            for thread_mode in threading:
                for map_name, map_params in maps.items():
                    # Create config name
                    config_name = f"batch_{batch_size}_{mode}_{thread_mode}_{map_name}"

                    # Create server config
                    multi_threading_bool = "true" if thread_mode == "multi" else "false"
                    server_content = server_template.format(
                        mode=mode,
                        multi_threading=multi_threading_bool,
                        batch_size=batch_size,
                        map_yaml_path=f"MAPS_DIR/{map_name}.yaml",
                        start_x=map_params["start_x"],
                        start_y=map_params["start_y"],
                        goal_x=map_params["goal_x"],
                        goal_y=map_params["goal_y"],
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

                    config_count += 1
                    print(f"Created config {config_count}: {config_name}")

    print(f"\nTotal configurations created: {config_count}")
    print(f"Configurations saved to: {config_dir}")
    print("\nNote: MAPS_DIR placeholder in server configs must be replaced")
    print("      with the actual installed map path before running experiments.")
    print("      The run script handles this automatically.")


if __name__ == "__main__":
    main()
