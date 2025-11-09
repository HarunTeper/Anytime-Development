# Video Publisher Separation - Update Summary

## Overview
Separated the video publisher into its own launch file to allow proper initialization of YOLO server/client components before image processing begins.

## Changes Made

### 1. New Launch File Created
**File:** `/home/vscode/workspace/packages/src/experiments/launch/video_publisher_only.launch.py`

A dedicated launch file for the video publisher that can be launched independently:
```python
ros2 launch experiments video_publisher_only.launch.py \
    image_path:=/path/to/images
```

**Features:**
- Single parameter: `image_path`
- Launches only the video publisher node
- Can be used standalone or with other components

### 2. Updated YOLO Launch File
**File:** `/home/vscode/workspace/packages/src/experiments/launch/yolo.launch.py`

**Removed:**
- Video publisher node
- `image_path` launch argument
- Related imports and configuration

**Result:**
- Now launches only the YOLO server and client components
- Cleaner separation of concerns
- Removed unused `Node` import

### 3. Updated All Experiment Scripts

All four YOLO experiment scripts now follow this pattern:

```bash
# 1. Start tracing
lttng start

# 2. Launch YOLO components
ros2 launch experiments yolo.launch.py \
    server_config:=... \
    client_config:=... &
YOLO_PID=$!

# 3. Wait for initialization (3 seconds)
sleep 3

# 4. Launch video publisher
ros2 launch experiments video_publisher_only.launch.py \
    image_path:=... &
VIDEO_PUB_PID=$!

# 5. Wait for completion
wait ${VIDEO_PUB_PID}

# 6. Cleanup both processes
kill ${VIDEO_PUB_PID}
kill ${YOLO_PID}
```

#### Updated Scripts:

1. **verify_tracepoints.sh**
   - Launches YOLO components first
   - Waits 3 seconds for initialization
   - Then launches video publisher
   - Tracks two PIDs: `YOLO_PID` and `VIDEO_PUB_PID`

2. **test_single_config.sh**
   - Same separation pattern
   - Monitors video publisher completion
   - Kills both processes on cleanup

3. **run_phase1_baseline.sh**
   - Launches with phase1 configs
   - 3-second initialization delay
   - Waits for video publisher to complete
   - Cleans up both processes

4. **run_phase3_max_throughput.sh**
   - Launches with phase3 config
   - 3-second initialization delay
   - Waits for video publisher to complete
   - Cleans up both processes

## Benefits

### 1. **Proper Initialization Order**
- YOLO server and client fully initialize before image processing
- Reduces startup race conditions
- Ensures action server is ready before first image arrives

### 2. **Better Process Control**
- Two separate PIDs to track
- Can monitor each component independently
- Easier debugging when issues occur

### 3. **Cleaner Architecture**
- Launch files have single responsibilities
- Video publisher can be reused in other experiments
- YOLO launch file focused only on anytime components

### 4. **Flexible Timing**
- Easy to adjust initialization delay (currently 3 seconds)
- Can be tuned per experiment if needed
- Clear separation between startup and processing phases

### 5. **Better Tracing**
- Tracing starts before any components
- Captures full initialization sequence
- No risk of missing early tracepoints

## Timing Sequence

```
Time   | Action
-------|-------------------------------------------------------
T+0s   | LTTng session created and tracing started
T+0s   | YOLO server & client launch
T+0-3s | Components initialize (action server ready)
T+3s   | Video publisher launches
T+3s+  | Image processing begins
T+end  | Video publisher completes and exits
T+end  | Stop tracing
T+end  | Kill both YOLO and video publisher processes
```

## Launch File Usage

### YOLO Components Only:
```bash
ros2 launch experiments yolo.launch.py \
    server_config:=/path/to/server.yaml \
    client_config:=/path/to/client.yaml
```

### Video Publisher Only:
```bash
ros2 launch experiments video_publisher_only.launch.py \
    image_path:=/path/to/images
```

### Combined (with delay):
```bash
# Launch YOLO
ros2 launch experiments yolo.launch.py &
YOLO_PID=$!

# Wait for initialization
sleep 3

# Launch video publisher
ros2 launch experiments video_publisher_only.launch.py &
VIDEO_PUB_PID=$!
```

## Verification

To verify the changes work correctly:

```bash
cd /home/vscode/workspace/experiments/yolo
./verify_tracepoints.sh
```

Expected behavior:
1. YOLO components launch first
2. 3-second pause
3. Video publisher launches
4. Images are processed
5. Both processes cleaned up properly

## Files Modified

✅ Created: `/home/vscode/workspace/packages/src/experiments/launch/video_publisher_only.launch.py`
✅ Updated: `/home/vscode/workspace/packages/src/experiments/launch/yolo.launch.py`
✅ Updated: `/home/vscode/workspace/experiments/yolo/verify_tracepoints.sh`
✅ Updated: `/home/vscode/workspace/experiments/yolo/test_single_config.sh`
✅ Updated: `/home/vscode/workspace/experiments/yolo/run_phase1_baseline.sh`
✅ Updated: `/home/vscode/workspace/experiments/yolo/run_phase3_max_throughput.sh`

All changes ensure proper component initialization order with a configurable delay!
