#!/usr/bin/env python3
"""
Generate all Monte Carlo experiment configuration files
"""

import os

# Configuration parameters
batch_sizes = [1024, 4096, 16384, 65536, 262144, 1048576, 4194304, 16777216]
modes = ["reactive", "proactive"]
threading = ["single", "multi"]

# Base directory
config_dir = "/home/vscode/workspace/experiments/monte_carlo/configs"

# Template for server config
server_template = """anytime_server:
  ros__parameters:
    # Anytime algorithm mode
    is_reactive_proactive: "{mode}"  # Options: "reactive", "proactive"
    
    # Threading configuration
    multi_threading: {multi_threading}  # Enable/disable multi-threading
    
    # Batch processing configuration
    batch_size: {batch_size}  # Number of iterations to compute per batch
    
    # Logging configuration
    log_level: "info"  # Options: "debug", "info", "warn", "error", "fatal"
"""

# Template for client config (same for all experiments)
client_template = """anytime_client:
  ros__parameters:
    # Goal timer configuration
    goal_timer_period_ms: 1000  # Period in milliseconds for the goal request timer
    
    # Cancel timeout configuration
    cancel_timeout_period_ms: 250  # Period in milliseconds for the cancel timeout timer
    
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
                # Create config name
                config_name = f"batch_{batch_size}_{mode}_{thread_mode}"

                # Create server config
                multi_threading_bool = "true" if thread_mode == "multi" else "false"
                server_content = server_template.format(
                    mode=mode,
                    multi_threading=multi_threading_bool,
                    batch_size=batch_size
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


if __name__ == "__main__":
    main()
