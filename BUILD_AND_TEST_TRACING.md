# Build and Test Instructions for Tracing Implementation

## Prerequisites

Ensure LTTng is installed:
```bash
sudo apt-get update
sudo apt-get install lttng-tools lttng-modules-dkms liblttng-ust-dev
```

## Build Order

The packages must be built in the correct dependency order:

1. **anytime_tracing** (base tracing infrastructure)
2. **anytime_core** (depends on anytime_tracing)
3. **anytime_monte_carlo** and **anytime_yolo** (depend on anytime_core)
4. **interference** (uses tracing directly)

### Build Commands

```bash
cd /home/vscode/workspace/packages

# Build only the tracing and core packages first
colcon build --packages-select anytime_tracing anytime_core

# Source the installation
source install/setup.bash

# Build the rest
colcon build --packages-select anytime_monte_carlo anytime_yolo interference

# Or build everything at once (colcon handles dependencies)
colcon build --symlink-install
```

## Testing Tracing

### 1. Verify Tracing is Enabled

```bash
# Source the workspace
source /home/vscode/workspace/packages/install/setup.bash

# Run a quick test to check if tracing is available
python3 << EOF
import ctypes
try:
    lib = ctypes.CDLL('libanytime_tracing.so')
    status = lib.anytime_trace_compile_status()
    print(f"Tracing enabled: {bool(status)}")
except Exception as e:
    print(f"Error: {e}")
EOF
```

### 2. Start LTTng Session

```bash
# Create a tracing session
lttng create anytime_test

# Enable all anytime tracepoints
lttng enable-event --userspace 'anytime:*'

# Start tracing
lttng start
```

### 3. Run a Simple Test

#### Monte Carlo Test:
```bash
# Terminal 1: Start Monte Carlo server
source /home/vscode/workspace/packages/install/setup.bash
ros2 launch anytime_monte_carlo action_server.launch.py

# Terminal 2: Start Monte Carlo client
source /home/vscode/workspace/packages/install/setup.bash
ros2 launch anytime_monte_carlo action_client.launch.py
```

#### YOLO Test (if GPU available):
```bash
# Terminal 1: Start YOLO server
source /home/vscode/workspace/packages/install/setup.bash
ros2 launch anytime_yolo action_server.launch.py

# Terminal 2: Start video publisher
source /home/vscode/workspace/packages/install/setup.bash
ros2 launch video_publisher video_publisher.launch.py

# Terminal 3: Start YOLO client
source /home/vscode/workspace/packages/install/setup.bash
ros2 launch anytime_yolo action_client.launch.py
```

### 4. Stop Tracing and View Results

```bash
# Stop tracing
lttng stop

# View trace events (text output)
babeltrace ~/.lttng-traces/anytime_test-*

# Or view with filtering
babeltrace ~/.lttng-traces/anytime_test-* | grep "anytime:monte_carlo"

# Destroy session when done
lttng destroy anytime_test
```

## Expected Tracepoints

### Monte Carlo Expected Events:
- `anytime:anytime_base_init`
- `anytime:anytime_server_init`
- `anytime:monte_carlo_init`
- `anytime:anytime_server_handle_goal`
- `anytime:anytime_server_handle_accepted`
- `anytime:anytime_base_activate`
- `anytime:reactive_anytime_function_entry` (or proactive variant)
- `anytime:anytime_compute_entry`
- `anytime:anytime_compute_iteration` (many times)
- `anytime:monte_carlo_iteration` (many times)
- `anytime:anytime_compute_exit`
- `anytime:anytime_calculate_result_entry`
- `anytime:monte_carlo_result`
- `anytime:anytime_calculate_result_exit`
- `anytime:anytime_send_feedback_entry`
- `anytime:anytime_send_feedback_exit`
- `anytime:reactive_anytime_function_exit` (or proactive variant)
- `anytime:anytime_base_deactivate`

### YOLO Expected Events:
- `anytime:anytime_base_init`
- `anytime:anytime_server_init`
- `anytime:yolo_init`
- `anytime:yolo_reset`
- `anytime:yolo_image_processed`
- `anytime:yolo_layer_start` (multiple times)
- `anytime:yolo_layer_end` (multiple times)
- `anytime:yolo_result`
- And all the base anytime events

### Client Expected Events:
- `anytime:anytime_client_init`
- `anytime:anytime_client_send_goal`
- `anytime:anytime_client_goal_response`
- `anytime:anytime_client_feedback` (multiple times)
- `anytime:anytime_client_result`

## Troubleshooting

### Compilation Errors

If you see errors about missing `anytime_tracing/anytime_tracetools.h`:
```bash
# Rebuild anytime_tracing first
cd /home/vscode/workspace/packages
colcon build --packages-select anytime_tracing
source install/setup.bash
# Then rebuild dependent packages
colcon build
```

### No Tracepoints Visible

If `babeltrace` shows no events:
1. Verify LTTng is properly installed: `lttng --version`
2. Check that tracing is enabled: `lttng list --userspace`
3. Ensure the session was started: `lttng list`
4. Verify the application ran: check ROS2 logs

### Tracing Disabled

If tracing was compiled without LTTng support:
1. Install LTTng development files: `sudo apt-get install liblttng-ust-dev`
2. Clean and rebuild: `colcon build --packages-select anytime_tracing --cmake-clean-cache`
3. Check CMake output for "LTTng found: anytime tracing enabled" message

## Advanced Analysis

### Using Trace Compass

1. Install Trace Compass: Download from https://www.eclipse.org/tracecompass/
2. Import trace: File → Open Trace → Select trace directory
3. View timeline, statistics, and custom analyses

### Custom Python Analysis

```python
import babeltrace

# Open trace
trace_collection = babeltrace.TraceCollection()
trace_collection.add_trace('~/.lttng-traces/anytime_test-XXXXXX', 'ctf')

# Iterate through events
for event in trace_collection.events:
    if 'monte_carlo' in event.name:
        print(f"{event.name}: {event.timestamp}")
        for field_name, field_value in event.items():
            print(f"  {field_name}: {field_value}")
```

## Next Steps

After verifying tracing works:
1. Create experiment configuration files (YAML)
2. Implement evaluation scripts to parse trace data
3. Create batch execution scripts for multiple runs
4. Generate plots and statistics from trace data

See `EXPERIMENTAL_PLAN.md` for the full experimental design.
