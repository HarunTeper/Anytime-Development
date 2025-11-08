# Custom Tracing Guide

This guide explains how to use the custom tracepoints defined in the `anytime_tracing` package.

## Summary

✅ **Custom tracepoints work perfectly with direct LTTng commands**  
✅ **No need for `ros2 trace` integration** - LTTng provides all necessary functionality  
✅ **Tracepoints are visible via `lttng list --userspace`**  
✅ **Can be enabled, captured, and analyzed using standard LTTng tools**

## Quick Start

### 1. Verify Installation

```bash
cd /home/vscode/workspace/packages
source install/setup.bash
```

### 2. Run a Complete Trace

Use this script to capture a trace with the custom tracepoint:

```bash
#!/bin/bash

# Source workspace
source /home/vscode/workspace/packages/install/setup.bash

# Create trace session
lttng create my-trace --output=/tmp/my-trace

# Enable custom tracepoint
lttng enable-event --userspace 'anytime:interference_timer_init'

# Start tracing
lttng start

# Run the application
timeout 5 ros2 run interference interference_timer_node &
wait

# Stop and view
lttng stop
lttng view | grep "anytime:"

# Cleanup
lttng destroy my-trace
```

## What Was Created

### 1. **anytime_tracing Package** (`packages/src/anytime_tracing/`)

A complete LTTng tracing library with:
- Custom tracepoint definitions
- LTTng integration
- C/C++ API for instrumentation

**Key Files:**
- `include/anytime_tracing/anytime_tracetools.h` - Main API header
- `include/anytime_tracing/tp_call.h` - LTTng tracepoint definitions
- `src/anytime_tracetools.c` - Tracepoint implementations
- `CMakeLists.txt` - Build configuration
- `package.xml` - ROS 2 package metadata

### 2. **Interference Package Integration**

The `interference` package now:
- Depends on `anytime_tracing`
- Calls `ANYTIME_TRACEPOINT()` in timer constructor
- Links against the tracing library

**Modified Files:**
- `packages/src/interference/CMakeLists.txt` - Added anytime_tracing dependency
- `packages/src/interference/package.xml` - Added anytime_tracing dependency  
- `packages/src/interference/src/interference_timer_node.cpp` - Added tracepoint call

## Custom Tracepoint: interference_timer_init

**Provider:** `anytime`  
**Event:** `interference_timer_init`  
**Full Name:** `anytime:interference_timer_init`

**Fields:**
- `node_handle` - Node handle pointer (hex)
- `timer_period_ms` - Timer period in milliseconds (integer)
- `execution_time_ms` - Busy-wait execution time in milliseconds (integer)
- `version` - Package version string

**Example Output:**
```
[01:18:01.584371195] anytime:interference_timer_init: { 
  cpu_id = 8, 
  node_handle = 0x62CD0C50FC50, 
  timer_period_ms = 1000, 
  execution_time_ms = 10, 
  version = "1.0.0" 
}
```

## Why We Don't Need ros2 trace Integration

The `ros2 trace --list` command only shows hardcoded ROS 2 tracepoints. However:

1. **LTTng provides all necessary functionality directly**
   - List events: `lttng list --userspace`
   - Enable events: `lttng enable-event --userspace 'anytime:*'`
   - Full control over sessions, channels, and filtering

2. **Custom tracepoints work perfectly with LTTng**
   - Automatically discovered by LTTng when applications load
   - Can be enabled/disabled dynamically
   - Full CTF (Common Trace Format) output support

3. **Standard tracing workflows work unchanged**
   - Create sessions
   - Enable events
   - Start/stop tracing
   - View and analyze traces

## Advanced Usage

### Enable Multiple Providers

```bash
lttng create session
lttng enable-event --userspace 'ros2:*'      # All ROS 2 events
lttng enable-event --userspace 'anytime:*'   # All custom events
lttng start
```

### Filter Events

```bash
# Only enable specific events
lttng enable-event --userspace 'anytime:interference_timer_init'
lttng enable-event --userspace 'ros2:rcl_timer_init'
```

### Add Context Information

```bash
lttng add-context --userspace --type vpid
lttng add-context --userspace --type vtid
lttng add-context --userspace --type procname
```

### Export to Trace Analysis Tools

The traces are in CTF format and can be analyzed with:
- **Trace Compass** - Eclipse-based trace viewer
- **babeltrace2** - Command-line trace converter
- **tracecompass-analysis** - ROS 2 specific trace analysis

```bash
# Convert to text
babeltrace2 /tmp/my-trace

# Analyze with trace compass
tracecompass /tmp/my-trace
```

## Testing Your Custom Tracepoints

Save this as `test_custom_trace.sh`:

```bash
#!/bin/bash
set -e

cd /home/vscode/workspace/packages
source install/setup.bash

echo "=== Custom Tracepoint Test ==="

# Cleanup
lttng destroy test 2>/dev/null || true

# Setup
lttng create test --output=/tmp/test-trace
lttng enable-event --userspace 'anytime:*'
lttng start

# Run
echo "Running interference node..."
timeout 3 ros2 run interference interference_timer_node \
  --ros-args -p timer_period_ms:=500 -p execution_time_ms:=5 &
wait

# Results
lttng stop
echo ""
echo "=== Captured Events ==="
lttng view | grep "anytime:" | head -5
echo ""
COUNT=$(lttng view | grep "anytime:" | wc -l)
echo "Total custom events: $COUNT"

# Cleanup
lttng destroy test
echo "=== Test Complete ==="
```

Then run:
```bash
chmod +x test_custom_trace.sh
./test_custom_trace.sh
```

## Troubleshooting

### Tracepoint Not Visible

**Problem:** `lttng list --userspace` doesn't show your tracepoint

**Solution:** The application must be running (or has run) to register tracepoints:
```bash
# Start your app first
ros2 run interference interference_timer_node &
# Then check
lttng list --userspace
```

### No Events Captured

**Problem:** Trace is empty

**Solution:** Make sure the event was enabled before starting the trace:
```bash
lttng create session
lttng enable-event --userspace 'anytime:interference_timer_init'  # Enable FIRST
lttng start  # Then start
# Run your app
```

### Permission Errors

**Problem:** Cannot create lttng session

**Solution:** Check if you're in the `tracing` group:
```bash
groups | grep tracing
# If not, add yourself:
sudo usermod -a -G tracing $USER
# Then logout and login
```

## Next Steps

1. Add more custom tracepoints for other components
2. Integrate tracing into your analysis pipeline
3. Create automated trace collection scripts
4. Use trace analysis tools like Trace Compass

For more information about LTTng, see: https://lttng.org/docs/
