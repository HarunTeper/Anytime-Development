#!/usr/bin/env python3
"""
Generate all Interference experiment configuration files
"""

import os

# Configuration parameters
batch_sizes = [1, 64, 4096, 16384, 65536, 262144, 524288, 1048576]
modes = ["reactive", "proactive"]
threading = ["single", "multi"]

# Interference timer fixed parameters
TIMER_PERIOD_MS = 100  # 100ms = 10Hz timer frequency
EXECUTION_TIME_MS = 10  # 10ms busy-wait per timer execution

# Base directory
config_dir = "/home/vscode/workspace/experiments/interference/configs"

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

# Template for interference config (same for all experiments)
interference_template = """interference_timer:
  ros__parameters:
    # Timer configuration
    timer_period_ms: {timer_period_ms}  # Period in milliseconds for the interference timer
    
    # Execution time (busy-wait duration)
    execution_time_ms: {execution_time_ms}  # Busy-wait duration in milliseconds
    
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


if __name__ == "__main__":
    main()
