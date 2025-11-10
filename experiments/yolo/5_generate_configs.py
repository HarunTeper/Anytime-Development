#!/usr/bin/env python3
"""
Step 5: Generate Cancellation Experiment Configurations

Creates configuration files for all combinations:
- Block sizes: 1, 8, 16, 25
- Mode: proactive
- Sync modes: sync, async
- Threading: single, multi

Total: 4 × 1 × 2 × 2 = 16 server configs + 1 client config

Client cancellation settings:
- Cancel after 16 layers
- Score threshold: 0.8
- Target class: 9 (traffic light)

Output: configs/phase4_*.yaml
"""

from pathlib import Path
import yaml

EXPERIMENT_DIR = Path("/home/vscode/workspace/experiments/yolo")
CONFIG_DIR = EXPERIMENT_DIR / "configs"

# Ensure config directory exists
CONFIG_DIR.mkdir(exist_ok=True)

# Configuration parameters
BLOCK_SIZES = [1, 8, 16, 25]
MODES = ["proactive"]
SYNC_MODES = ["sync", "async"]
THREADING_MODES = ["single", "multi"]

# Client configuration (same for all)
CLIENT_CONFIG = {
    "anytime_client": {
        "ros__parameters": {
            "image_topic": "video_frames",
            "cancel_after_layers": 16,  # Cancel after 16 layers
            "cancel_layer_score": True,  # Enable score-based cancellation
            "score_threshold": 0.8,       # Threshold of 0.8
            "target_class_id": "9",       # Traffic light
            "log_level": "info"
        }
    }
}

# Weights path
WEIGHTS_PATH = "/home/vscode/workspace/packages/src/anytime_yolo/weights_32"


def create_server_config(block_size, mode, sync_mode, threading_mode):
    """Create server configuration dictionary"""
    return {
        "anytime_server": {
            "ros__parameters": {
                "is_reactive_proactive": mode,
                "multi_threading": threading_mode == "multi",
                "batch_size": block_size,
                "is_sync_async": sync_mode,
                "weights_path": WEIGHTS_PATH,
                "log_level": "info"
            }
        }
    }


def main():
    print("="*80)
    print("STEP 5: GENERATE CANCELLATION CONFIGS")
    print("="*80)
    print(f"\nOutput: {CONFIG_DIR}")
    print(f"\nBlock sizes: {BLOCK_SIZES}")
    print(f"Modes: {MODES}")
    print(f"Sync modes: {SYNC_MODES}")
    print(f"Threading modes: {THREADING_MODES}")

    # Create client config (shared)
    client_path = CONFIG_DIR / "phase4_client.yaml"
    with open(client_path, 'w') as f:
        yaml.dump(CLIENT_CONFIG, f, default_flow_style=False, sort_keys=False)
    print(f"\n✓ Created: {client_path}")

    # Generate all server configurations
    config_count = 0
    for block_size in BLOCK_SIZES:
        for mode in MODES:
            for sync_mode in SYNC_MODES:
                for threading_mode in THREADING_MODES:
                    # Create server config
                    server_config = create_server_config(
                        block_size, mode, sync_mode, threading_mode
                    )

                    # Generate filename
                    filename = f"phase4_server_bs{block_size}_{mode}_{sync_mode}_{threading_mode}.yaml"
                    filepath = CONFIG_DIR / filename

                    # Write config
                    with open(filepath, 'w') as f:
                        yaml.dump(server_config, f,
                                  default_flow_style=False, sort_keys=False)

                    config_count += 1
                    print(f"✓ Created: {filename}")

    print(f"\n" + "="*80)
    print(f"✅ STEP 5 COMPLETE: CONFIGS GENERATED")
    print("="*80)
    print(
        f"Created {config_count} server configs + 1 client config = {config_count + 1} files")

    # Print summary
    print("\nConfiguration Summary:")
    print(f"  Block sizes: {len(BLOCK_SIZES)}")
    print(f"  Modes: {len(MODES)}")
    print(f"  Sync modes: {len(SYNC_MODES)}")
    print(f"  Threading modes: {len(THREADING_MODES)}")
    print(
        f"  Total: {len(BLOCK_SIZES) * len(MODES) * len(SYNC_MODES) * len(THREADING_MODES)} combinations")

    print("\nClient Cancellation Settings:")
    print(f"  Cancel after: 16 layers")
    print(f"  Score threshold: ≥ 0.8")
    print(f"  Target class: 9 (traffic light)")

    print("\nNext step:")
    print(f"  6. Run experiments: ./6_run_experiments.sh")


if __name__ == "__main__":
    main()
