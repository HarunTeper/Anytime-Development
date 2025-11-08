# Tracing Quick Reference Card

## Quick Commands

### Start Tracing
```bash
lttng create my_session
lttng enable-event --userspace 'anytime:*'
lttng start
```

### Run Experiments
```bash
source /home/vscode/workspace/packages/install/setup.bash
# Run your ROS2 nodes...
```

### Stop and View
```bash
lttng stop
babeltrace ~/.lttng-traces/my_session-* | less
lttng destroy my_session
```

## Tracepoint Categories

| Category | Count | Purpose |
|----------|-------|---------|
| AnytimeBase | 14 | Core execution flow and timing |
| AnytimeServer | 4 | Server-side action handling |
| AnytimeClient | 7 | Client-side requests and responses |
| Monte Carlo | 4 | Domain-specific Monte Carlo events |
| YOLO | 10 | Neural network layer and detection events |
| Interference | 3 | Timer interference analysis |
| **TOTAL** | **41** | **Complete system coverage** |

## Common Tracepoint Patterns

### Timing a Function
```
anytime:function_entry    ← Start timestamp
  ... function execution ...
anytime:function_exit     ← End timestamp
```

### Batch Processing
```
anytime:compute_entry (batch_size=10)
  anytime:compute_iteration (i=0)
  anytime:compute_iteration (i=1)
  ...
  anytime:compute_iteration (i=9)
anytime:compute_exit (time=1234567 ns)
```

### Full Request Flow
```
Client Side:                    Server Side:
anytime:client_send_goal    →   anytime:server_handle_goal
anytime:client_goal_response ← anytime:server_handle_accepted
                                anytime:base_activate
                                anytime:reactive_function_entry
                                  anytime:compute_entry
                                    ... iterations ...
                                  anytime:compute_exit
                                  anytime:send_feedback_entry
anytime:client_feedback      ←  anytime:send_feedback_exit
                                  ... more iterations ...
                                anytime:reactive_function_exit
                                anytime:base_deactivate
anytime:client_result        ←  (goal completed)
```

## Filtering Traces

### View Only Monte Carlo Events
```bash
babeltrace ~/.lttng-traces/my_session-* | grep "monte_carlo"
```

### View Only Timing Events
```bash
babeltrace ~/.lttng-traces/my_session-* | grep -E "(entry|exit)"
```

### View Specific Node
```bash
babeltrace ~/.lttng-traces/my_session-* | grep "node_handle.*0x12345"
```

### Extract to CSV
```bash
babeltrace --fields=all ~/.lttng-traces/my_session-* > trace.csv
```

## Python Analysis Template

```python
import babeltrace

trace = babeltrace.TraceCollection()
trace.add_trace('/path/to/trace', 'ctf')

# Find compute times
compute_times = []
compute_start = None

for event in trace.events:
    if event.name == 'anytime:anytime_compute_entry':
        compute_start = event.timestamp
    elif event.name == 'anytime:anytime_compute_exit':
        if compute_start:
            duration = event.timestamp - compute_start
            compute_times.append(duration)
            compute_start = None

print(f"Average compute time: {sum(compute_times)/len(compute_times)} ns")
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| No events visible | Check `lttng list` shows events enabled |
| Compilation error | Rebuild anytime_tracing first |
| Missing tracepoints | Verify LTTng installed: `lttng --version` |
| Performance issues | Tracing overhead is < 1%, check other causes |

## File Locations

| Component | Path |
|-----------|------|
| Tracepoint definitions | `packages/src/anytime_tracing/include/anytime_tracing/tp_call.h` |
| Helper macros (core) | `packages/src/anytime_core/include/anytime_core/tracing.hpp` |
| Helper macros (MC) | `packages/src/anytime_monte_carlo/include/anytime_monte_carlo/tracing.hpp` |
| Helper macros (YOLO) | `packages/src/anytime_yolo/include/anytime_yolo/tracing.hpp` |
| Full documentation | `TRACING_IMPLEMENTATION.md` |
| Build guide | `BUILD_AND_TEST_TRACING.md` |

## Key Metrics to Extract

### Monte Carlo
- Iterations per batch: Count `monte_carlo_iteration` between `compute_entry/exit`
- Batch processing time: Subtract `compute_entry` from `compute_exit` timestamps
- Cancellation delay: Time from cancel request to `base_deactivate`

### YOLO
- Layer execution time: Subtract `layer_start` from `layer_end` timestamps
- Detections per layer: Parse `yolo_detection` events, group by layer_num
- Total throughput: Count images processed per unit time

### Interference
- Timer jitter: Compare expected vs actual periods in `callback_exit` events
- Task blocking: Measure gaps between `function_entry` and actual execution
- Deadline misses: Count `callback_exit` events with duration > target

## Advanced Features

### Conditional Tracing
```bash
# Only trace Monte Carlo
lttng enable-event --userspace 'anytime:monte_carlo_*'

# Only trace timing events
lttng enable-event --userspace 'anytime:*_entry'
lttng enable-event --userspace 'anytime:*_exit'
```

### High-Frequency Tracing
```bash
# Increase buffer size for high-frequency events
lttng create my_session --output=/path/to/large/disk
lttng enable-channel --userspace big_channel --subbuf-size=8M --num-subbuf=32
lttng enable-event --userspace --channel=big_channel 'anytime:*'
```

### Live Viewing
```bash
# Start live trace viewing (different terminal)
lttng create --live
lttng enable-event --userspace 'anytime:*'
lttng start

# In another terminal
babeltrace --input-format=lttng-live net://localhost
```

---

**For complete details, see**: `TRACING_IMPLEMENTATION.md` and `BUILD_AND_TEST_TRACING.md`
