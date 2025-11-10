# anytime_tracing

Custom LTTng tracepoints for anytime system performance analysis.

## Overview

Provides low-overhead tracing for:
- Anytime base computation (batch timing, iterations)
- Server/client action lifecycle
- Monte Carlo domain events
- YOLO layer processing and detections
- Interference timer callbacks

## Usage

```bash
# Create and start tracing session
lttng create my_session
lttng enable-event --userspace 'anytime:*'
lttng add-context --userspace --type=vpid --type=vtid
lttng start

# Run your application
ros2 launch experiments monte_carlo.launch.py

# Stop and view traces
lttng stop
babeltrace ~/.lttng-traces/my_session-*/
lttng destroy
```

## Key Tracepoints

- `anytime_compute_entry/exit` - Batch computation timing
- `anytime_compute_iteration` - Individual iteration tracking
- `client_send_goal` - Goal request timing
- `yolo_layer_start/end` - Layer processing
- `yolo_exit_calculation_end` - Detection results
- `interference_timer_callback` - Timer interference

See `include/anytime_tracing/tp_call.h` for all tracepoint definitions.
