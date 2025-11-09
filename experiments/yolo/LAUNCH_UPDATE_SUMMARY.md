# YOLO Experiment Scripts - Launch File Update Summary

## Overview
Updated all YOLO experiment shell scripts to use the unified `experiments` package launch file instead of individual component launch files.

## Changes Made

### 1. Launch File Migration
**Old approach:**
- Launched three separate components:
  - `ros2 launch anytime_yolo action_client.launch.py`
  - `ros2 launch anytime_yolo action_server.launch.py`
  - `ros2 launch video_publisher video_publisher.launch.py`
- Parameters passed as command-line arguments

**New approach:**
- Single unified launch file:
  - `ros2 launch experiments yolo.launch.py`
- Parameters configured via YAML files
- All components launched together in a managed container

### 2. Parameter Configuration

#### Available Launch Arguments
The new `yolo.launch.py` accepts these parameters:
- `client_config` - Path to client YAML configuration
- `server_config` - Path to server YAML configuration  
- `container_name` - Name of the component container
- `log_level` - Logging level (debug, info, warn, error, fatal)
- `use_multi_threaded` - Executor type (true/false)
- `image_path` - Path to image directory

#### Server Parameters (in YAML)
```yaml
anytime_server:
  ros__parameters:
    is_reactive_proactive: "proactive"  # or "reactive"
    multi_threading: true
    batch_size: 1  # Number of layers per batch
    is_sync_async: "sync"  # or "async"
    weights_path: "/path/to/weights"
    log_level: "info"
```

#### Client Parameters (in YAML)
```yaml
anytime_client:
  ros__parameters:
    image_topic: "video_frames"
    cancel_after_layers: 12
    cancel_layer_score: false
    score_threshold: 0.7
    target_class_id: "9"
    log_level: "info"
```

### 3. Updated Scripts

#### verify_tracepoints.sh
- Now uses `experiments` launch file
- Single process to manage instead of 3
- Cleaner cleanup logic

#### test_single_config.sh
- Simplified to single launch command
- Uses default configuration (batch_size=1, proactive mode)
- Easier to maintain and debug

#### run_phase1_baseline.sh
- Uses default configuration
- Single launch process per trial
- Consistent with Phase 1 goals (baseline measurement)

#### run_phase3_max_throughput.sh
- Uses custom config: `configs/phase3_server.yaml`
- Sets `batch_size: 25` for maximum throughput
- Custom config created specifically for Phase 3

### 4. Configuration Files Created

#### experiments/yolo/configs/phase3_server.yaml
```yaml
anytime_server:
  ros__parameters:
    is_reactive_proactive: "proactive"
    multi_threading: true
    batch_size: 25  # All layers at once
    is_sync_async: "sync"
    weights_path: "/home/vscode/workspace/packages/src/anytime_yolo/weights_32"
    log_level: "info"
```

## Benefits

1. **Unified Architecture**: All components managed in a single container
2. **Configuration Management**: Parameters in YAML files (version controllable)
3. **Cleaner Scripts**: Fewer processes to manage
4. **Better Process Control**: Single PID to track and kill
5. **Consistency**: Same launch mechanism across all experiments
6. **Flexibility**: Easy to create custom configs for different phases

## Usage Examples

### Basic usage with defaults:
```bash
ros2 launch experiments yolo.launch.py
```

### With custom server config:
```bash
ros2 launch experiments yolo.launch.py \
    server_config:=/path/to/custom_server.yaml
```

### With custom image path:
```bash
ros2 launch experiments yolo.launch.py \
    image_path:=/path/to/images
```

### Full customization:
```bash
ros2 launch experiments yolo.launch.py \
    server_config:=/path/to/server.yaml \
    client_config:=/path/to/client.yaml \
    image_path:=/path/to/images \
    log_level:=debug
```

## Migration Checklist

- [x] Update verify_tracepoints.sh
- [x] Update test_single_config.sh  
- [x] Update run_phase1_baseline.sh
- [x] Update run_phase3_max_throughput.sh
- [x] Create phase3_server.yaml config
- [x] Remove invalid launch parameters
- [x] Simplify cleanup logic
- [x] Test scripts work with new launch file

## Notes

- Default configs use `batch_size: 1` and `proactive` mode
- Phase 3 requires custom config with `batch_size: 25`
- All scripts now properly handle single process cleanup
- Video publisher is integrated into the main launch file
- Tracepoints remain unchanged - same data collection as before
