#!/usr/bin/env python3
"""
Step 0: Generate Baseline Configurations

Creates configuration files needed for Step 1 (baseline data collection):
- phase1_server.yaml: Server config with batch_size=1, proactive, sync, single-threaded
- phase1_client.yaml: Client config with no cancellation (processes all 25 layers)

These configs are used to collect baseline quality and timing data.

Output: configs/phase1_*.yaml
"""

from pathlib import Path
import yaml

EXPERIMENT_DIR = Path("/home/vscode/workspace/experiments/yolo")
CONFIG_DIR = EXPERIMENT_DIR / "configs"

# Ensure config directory exists
CONFIG_DIR.mkdir(exist_ok=True)

# Weights path
WEIGHTS_PATH = "/home/vscode/workspace/packages/src/anytime_yolo/weights_32"


def create_phase1_configs():
    """Create Phase 1 baseline configuration files"""

    # Server configuration: batch_size=1, proactive, sync, single-threaded
    server_config = {
        "anytime_server": {
            "ros__parameters": {
                "is_reactive_proactive": "proactive",
                "multi_threading": False,
                "batch_size": 1,  # Process one layer at a time
                "is_sync_async": "sync",
                "weights_path": WEIGHTS_PATH,
                "log_level": "info"
            }
        }
    }

    # Client configuration: no cancellation, process all layers
    client_config = {
        "anytime_client": {
            "ros__parameters": {
                "image_topic": "video_frames",
                "cancel_after_layers": 25,  # Don't cancel, process all layers
                "cancel_layer_score": False,  # Disable score-based cancellation
                "score_threshold": 0.0,       # Not used
                # Traffic light (for future reference)
                "target_class_id": "9",
                "log_level": "info"
            }
        }
    }

    # Write server config
    server_path = CONFIG_DIR / "phase1_server.yaml"
    with open(server_path, 'w') as f:
        yaml.dump(server_config, f, default_flow_style=False, sort_keys=False)
    print(f"✓ Created: {server_path}")

    # Write client config
    client_path = CONFIG_DIR / "phase1_client.yaml"
    with open(client_path, 'w') as f:
        yaml.dump(client_config, f, default_flow_style=False, sort_keys=False)
    print(f"✓ Created: {client_path}")


def create_phase3_configs():
    """Create Phase 3 throughput configuration files"""

    configs_created = []

    # Configurations: sync/async × single/multi-threaded
    for sync_mode in ["sync", "async"]:
        for threading_mode in ["single", "multi"]:
            multi_threading = (threading_mode == "multi")

            # Server configuration: batch_size=25, proactive, no cancellation
            server_config = {
                "anytime_server": {
                    "ros__parameters": {
                        "is_reactive_proactive": "proactive",
                        "multi_threading": multi_threading,
                        "batch_size": 25,  # All layers at once
                        "is_sync_async": sync_mode,
                        "weights_path": WEIGHTS_PATH,
                        "log_level": "info"
                    }
                }
            }

            # Write server config
            filename = f"phase3_server_{sync_mode}_{threading_mode}.yaml"
            filepath = CONFIG_DIR / filename
            with open(filepath, 'w') as f:
                yaml.dump(server_config, f,
                          default_flow_style=False, sort_keys=False)
            configs_created.append(filename)

    # Client configuration: no cancellation, process all layers
    client_config = {
        "anytime_client": {
            "ros__parameters": {
                "image_topic": "video_frames",
                "cancel_after_layers": 25,  # Don't cancel, process all layers
                "cancel_layer_score": False,  # Disable score-based cancellation
                "score_threshold": 0.0,
                "target_class_id": "9",
                "log_level": "info"
            }
        }
    }

    client_path = CONFIG_DIR / "phase3_client.yaml"
    with open(client_path, 'w') as f:
        yaml.dump(client_config, f, default_flow_style=False, sort_keys=False)
    configs_created.append("phase3_client.yaml")

    for filename in configs_created:
        print(f"✓ Created: {CONFIG_DIR / filename}")


def main():
    print("="*80)
    print("GENERATE BASELINE & THROUGHPUT CONFIGS")
    print("="*80)
    print(f"\nOutput: {CONFIG_DIR}")
    print()

    # Create Phase 1 configs
    print("Phase 1 (Baseline) Configurations:")
    print("  - Batch size: 1")
    print("  - Mode: Proactive")
    print("  - Sync: sync")
    print("  - Threading: single")
    print("  - Cancellation: disabled (all 25 layers)")
    print()
    create_phase1_configs()

    print()

    # Create Phase 3 configs
    print("Phase 3 (Throughput) Configurations:")
    print("  - Batch size: 25")
    print("  - Mode: Proactive")
    print("  - Sync modes: sync, async")
    print("  - Threading modes: single, multi")
    print("  - Cancellation: disabled (all 25 layers)")
    print()
    create_phase3_configs()

    print()
    print("="*80)
    print("✅ BASELINE & THROUGHPUT CONFIGS GENERATED")
    print("="*80)
    print(f"Created 7 configuration files in {CONFIG_DIR}")
    print()
    print("Next steps:")
    print("  1. Collect baseline: ./1_collect_baseline.sh")
    print("  2a. Analyze quality: python3 2a_analyze_quality.py")
    print("  2b. Analyze blocks: python3 2b_analyze_blocks.py")
    print("  3. Measure throughput: ./3_measure_throughput.sh")


if __name__ == "__main__":
    main()
