# YOLO Experiment Scripts - Configuration Reference

## Fixed Issues

### Issue 1: Empty weights_path causing JSON parse error
**Problem:** The default server config had an empty `weights_path: ""` which caused:
```
[json.exception.parse_error.101] parse error at line 1, column 1: 
syntax error while parsing value - unexpected end of input
```

**Solution:** Updated all config files to have explicit weights path:
```yaml
weights_path: "/home/vscode/workspace/packages/src/anytime_yolo/weights_32"
```

**Files Fixed:**
- `/home/vscode/workspace/packages/src/experiments/config/yolo/default_server.yaml`

## Script Configuration Summary

### 1. verify_tracepoints.sh
**Purpose:** Quick verification test for tracepoints
**Config Used:** Default configs (from experiments package)
- Server: `experiments/config/yolo/default_server.yaml`
- Client: `experiments/config/yolo/default_client.yaml`
**Settings:**
- Batch size: 1
- Mode: Proactive
- Multi-threading: true
- Weights: `/home/vscode/workspace/packages/src/anytime_yolo/weights_32`

### 2. test_single_config.sh
**Purpose:** Test single configuration to verify setup
**Config Used:** Default configs (from experiments package)
- Server: `experiments/config/yolo/default_server.yaml`
- Client: `experiments/config/yolo/default_client.yaml`
**Settings:**
- Batch size: 1
- Mode: Proactive
- Multi-threading: true
- Weights: `/home/vscode/workspace/packages/src/anytime_yolo/weights_32`

### 3. run_phase1_baseline.sh
**Purpose:** Phase 1 baseline experiments - collect layer-wise quality data
**Config Used:** Phase 1 specific configs
- Server: `experiments/yolo/configs/phase1_server.yaml`
- Client: `experiments/yolo/configs/phase1_client.yaml`
**Settings:**
- Batch size: 1
- Mode: Proactive
- Multi-threading: **false** (for baseline measurements)
- Cancellation: Disabled (cancel_after_layers: 25)
- Weights: `/home/vscode/workspace/packages/src/anytime_yolo/weights_32`

### 4. run_phase3_max_throughput.sh
**Purpose:** Phase 3 maximum throughput experiments
**Config Used:** Phase 3 specific configs
- Server: `experiments/yolo/configs/phase3_server.yaml`
- Client: Default (`experiments/config/yolo/default_client.yaml`)
**Settings:**
- Batch size: **25** (all layers at once)
- Mode: Proactive
- Multi-threading: true
- Cancellation: Disabled (cancel_after_layers: 12, but won't trigger with batch_size=25)
- Weights: `/home/vscode/workspace/packages/src/anytime_yolo/weights_32`

## Configuration Files

### Default Configs (in experiments package)
Location: `/home/vscode/workspace/packages/src/experiments/config/yolo/`

1. **default_server.yaml**
   - batch_size: 1
   - is_reactive_proactive: "proactive"
   - is_sync_async: "sync"
   - multi_threading: true
   - weights_path: "/home/vscode/workspace/packages/src/anytime_yolo/weights_32"

2. **default_client.yaml**
   - image_topic: "video_frames"
   - cancel_after_layers: 12
   - cancel_layer_score: false
   - score_threshold: 0.7
   - target_class_id: "9"

### Phase-Specific Configs (in experiments/yolo)
Location: `/home/vscode/workspace/experiments/yolo/configs/`

1. **phase1_server.yaml**
   - batch_size: 1
   - multi_threading: **false**
   - Everything else same as default

2. **phase1_client.yaml**
   - cancel_after_layers: **25** (disabled)
   - Everything else same as default

3. **phase3_server.yaml**
   - batch_size: **25**
   - multi_threading: true
   - Everything else same as default

## Verification Commands

### Check if weights exist:
```bash
ls -la /home/vscode/workspace/packages/src/anytime_yolo/weights_32/
```

### Test launch file directly:
```bash
ros2 launch experiments yolo.launch.py \
    image_path:=/home/vscode/workspace/packages/src/video_publisher/images
```

### Test with specific configs:
```bash
ros2 launch experiments yolo.launch.py \
    server_config:=/home/vscode/workspace/experiments/yolo/configs/phase1_server.yaml \
    client_config:=/home/vscode/workspace/experiments/yolo/configs/phase1_client.yaml \
    image_path:=/home/vscode/workspace/packages/src/video_publisher/images
```

### Verify tracepoints in output:
```bash
babeltrace2 /path/to/trace/dir | grep 'anytime:yolo' | head -20
```

## All Scripts Status

✅ **verify_tracepoints.sh** - Fixed (uses default config with correct weights_path)
✅ **test_single_config.sh** - Fixed (uses default config with correct weights_path)
✅ **run_phase1_baseline.sh** - Fixed (uses phase1 configs with correct weights_path)
✅ **run_phase3_max_throughput.sh** - Fixed (uses phase3 config with correct weights_path)

All scripts now properly configured and ready to use!
