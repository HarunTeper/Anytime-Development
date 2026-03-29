# Interference RRT* Experiment -- Methodology

## 1. Research Question

**How does the block size of an anytime RRT* path planner affect the timing accuracy of a co-located periodic callback sharing the same single-threaded ROS 2 executor?**

In a single-threaded executor, all callbacks (timers, action server work, subscriptions) run sequentially on one thread. A compute-heavy callback that holds the executor for longer than the timer period will delay or skip subsequent timer firings. This experiment quantifies that effect across a range of block sizes and two anytime scheduling modes (reactive vs. proactive).

### Why This Matters
Real-time robotics systems often host multiple nodes in a single process for low-latency intra-process communication. Understanding how block computation interferes with periodic tasks (e.g., control loops, sensor polling) is critical for selecting safe block sizes that keep jitter within acceptable bounds.

## 2. System Under Test

### 2.1 Architecture

Three ROS 2 composable nodes are loaded into component containers managed by a single-threaded executor:

```
┌─────────────────────────────────────────────────────┐
│  Server Container (single-threaded executor)         │
│                                                      │
│  ┌──────────────────────┐  ┌──────────────────────┐ │
│  │  AnytimeRrtAction    │  │  InterferenceTimer   │ │
│  │  Server              │  │  Node                │ │
│  │                      │  │                      │ │
│  │  - Waitable triggers │  │  - 100 ms wall timer │ │
│  │    block computation │  │  - 10 ms busy-wait   │ │
│  │  - block_size iters  │  │    per callback      │ │
│  │    per activation    │  │                      │ │
│  └──────────────────────┘  └──────────────────────┘ │
└─────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────┐
│  Client Container (single-threaded executor)         │
│                                                      │
│  ┌──────────────────────┐                            │
│  │  AnytimeRrtAction    │                            │
│  │  Client              │                            │
│  │                      │                            │
│  │  - Sends goals every │                            │
│  │    500 ms            │                            │
│  │  - Cancels after     │                            │
│  │    200 ms            │                            │
│  └──────────────────────┘                            │
└─────────────────────────────────────────────────────┘
```

The server and interference timer share the **same** single-threaded executor. The client is in a separate container so its timers do not interfere with measurement.

### 2.2 RRT* Server: How Block Computation Works

The server uses the anytime framework's waitable pattern to yield to the executor between blocks:

1. **Goal arrives** from the client. The server resets the RRT* tree and activates the waitable.
2. **Waitable triggers** → executor calls the anytime function (reactive or proactive).
3. **`compute()` executes `block_size` iterations** of the RRT* algorithm:
   - Each iteration: sample random point (with goal bias) → find nearest node → steer → collision check → find nearby nodes → choose best parent → insert node → rewire nearby nodes → check goal reached → periodic pruning.
   - The `anytime_compute_entry` tracepoint fires before the loop; `anytime_compute_exit` fires after.
   - Within the loop, if a cancellation is detected, the function returns early **without** emitting `anytime_compute_exit`. This produces unpaired entry events that the evaluation script must handle (see Section 9.2).
4. **After the block**, the anytime function sends feedback and calls `notify_waitable()`, which triggers a guard condition.
5. **Executor resumes** → processes any ready callbacks (the interference timer if its period has elapsed) → returns to the waitable when it fires again.

This means the timer can only execute **between blocks**. If a single block takes longer than 100 ms (the timer period), one or more timer firings will be delayed or skipped entirely.

### 2.3 Reactive vs. Proactive Mode

Both modes execute the same `compute()` block loop. They differ in what happens after:

| | Reactive | Proactive |
|---|----------|-----------|
| **After block** | Check cancel → if cancel or done: calculate result, finish goal | Check cancel → if cancel or done: finish goal (no result calc) |
| **If continuing** | Send feedback, notify waitable | Calculate result, send feedback, notify waitable |
| **Cancel response** | Immediate: result calculated only when stopping | Slightly delayed: result already calculated each cycle |
| **Executor occupancy** | Less work between blocks → more timer opportunities | More work per cycle (result calc every time) → fewer timer opportunities |

### 2.4 Interference Timer Node

The interference timer simulates a periodic real-time task:

- **Timer type**: ROS 2 wall timer (`create_wall_timer`), which uses wall-clock time.
- **Period**: 100 ms (configurable via `timer_period_ms` parameter).
- **Callback behavior**: Busy-waits for `execution_time_ms` (default 10 ms) using a tight `std::chrono::steady_clock` spin loop. This simulates a non-trivial callback workload.
- **Tracepoints**: `interference_timer_callback_entry` fires at callback start (before busy-wait), `interference_timer_callback_exit` fires at callback end (with measured duration in nanoseconds).

### 2.5 Client Goal/Cancel Cycle

The client drives the server through repeated goal/cancel cycles:

1. **Goal timer fires** (every `goal_timer_period_ms` = 500 ms): sends an RRT* goal requesting 100,000 iterations.
2. **Goal accepted**: starts the cancel timeout timer.
3. **Cancel timeout fires** (after `cancel_timeout_period_ms` = 200 ms): sends `async_cancel_goal`.
4. **Result received**: resets goal timer, cycle repeats.

This produces a repeating pattern: 200 ms of active computation → cancel → brief idle → next goal. The server never completes 100,000 iterations; it always gets cancelled, which is the intended anytime behavior.

### 2.6 ROS 2 Wall Timer Semantics

When the executor dispatches a timer callback, it advances the timer's `next_call_time` by one period from the original schedule. If the executor was blocked for N periods, `next_call_time` jumps forward by N periods — but only **one** callback is dispatched. The other N-1 firings are silently skipped (never queued or dispatched). This is the "catch-up" behavior we detect and count as skipped firings.

## 3. Experimental Design

### 3.1 Independent Variables

| Variable | Values | Rationale |
|----------|--------|-----------|
| **Block size** | 1, 16, 64, 256, 1024, 4096, 16384 | Spans from negligible (<1 ms) to multi-hundred-ms compute per block, crossing the 100 ms timer period threshold |
| **Anytime mode** | reactive, proactive | Tests whether the scheduling mode's per-cycle overhead difference is measurable in timer jitter |

Total configurations: 7 × 2 = 14.

### 3.2 Dependent Variables

| Variable | Type | Description |
|----------|------|-------------|
| **Timer period** | Primary | Time between consecutive `interference_timer_callback_entry` events (ms) |
| **Jitter** | Primary | `actual_period − 100 ms`. Positive = delayed |
| **Max absolute jitter** | Primary | Worst-case `|jitter|` per run |
| **Skipped firings** | Primary | Timer firings the executor never dispatched due to block blocking |
| **Compute block time** | Secondary | Duration between `anytime_compute_entry` and `anytime_compute_exit` (ms) |

### 3.3 Controlled Variables

| Variable | Value | Notes |
|----------|-------|-------|
| Executor type | Single-threaded | Multi-threaded would eliminate interference |
| Timer period | 100 ms (10 Hz) | |
| Timer busy-wait | 10 ms | Simulates non-trivial callback workload |
| Map | depot.yaml | 604×307 px, 0.05 m/px, origin [0, 0, 0] |
| Start | (5.0, 12.0) m | |
| Goal | (25.0, 2.0) m | |
| step_size | 0.5 | |
| goal_threshold | 0.5 | |
| goal_bias | 0.05 | |
| gamma_rrt_star | 0.0 (auto) | Computed from map free area at runtime |
| prune_interval | 1000 | |
| random_seed | 42 | Deterministic tree construction |
| Client goal period | 500 ms | |
| Client cancel timeout | 200 ms | |
| Run duration | 10 s (5 s quick mode) | |
| Runs per config | 5 (1 in quick mode) | |

## 4. Configuration Generation

**Script**: `generate_configs.py`

For each of the 14 (block_size, mode) combinations, produces three YAML files in `configs/`:

| File pattern | ROS 2 node name | Content |
|-------------|-----------------|---------|
| `block_{size}_{mode}_single_server.yaml` | `anytime_server` | Algorithm mode, block_size, map path (`MAPS_DIR/depot.yaml` placeholder), start/goal coordinates, RRT* parameters, random seed |
| `block_{size}_{mode}_single_client.yaml` | `anytime_client` | goal_timer_period_ms=500, cancel_timeout_period_ms=200 |
| `block_{size}_{mode}_single_interference.yaml` | `interference_timer` | timer_period_ms=100, execution_time_ms=10 |

Config files contain only parameters that are declared and read by their respective nodes.
Threading is controlled by the launch file's executor type (`component_container` vs.
`component_container_mt`), not by a node parameter. Log level is set via container-level
`--ros-args --log-level` arguments, not via YAML parameters.

The `MAPS_DIR` placeholder is replaced at runtime by the shell scripts:
```bash
MAPS_DIR="$(ros2 pkg prefix anytime_rrt_star)/share/anytime_rrt_star/maps"
find configs/ -name "*_server.yaml" -exec sed -i "s|MAPS_DIR|${MAPS_DIR}|g" {} \;
```

The ROS 2 node name in each YAML's top-level key (e.g., `interference_timer:`) must match the `name=` argument in the launch file's `ComposableNode` declaration. This is how ROS 2 routes YAML parameters to the correct node.

## 5. Launch Architecture

**File**: `interference_rrt_star.launch.py`

The launch file accepts three config paths (`server_config`, `client_config`, `interference_config`) and creates:

- **Server container** (`component_container`, single-threaded): loads `AnytimeRrtActionServer` + `InterferenceTimerNode` as composable nodes. Both share the same executor thread.
- **Client container** (`component_container`, single-threaded): loads `AnytimeRrtActionClient` in isolation.

All three configs are passed as `parameters=[config_path]` to their respective `ComposableNode` declarations. The launch file defaults to single-threaded (`use_multi_threaded='false'`) to match the experiment design.

## 6. Execution Pipeline

**Scripts**: `run_interference_experiments.sh` (full), `run_quick.sh` (subset), `test_single_config.sh` (verification)

### 6.1 Pre-Experiment Setup

1. **Preflight**: Verify `lttng` is installed.
2. **Restart `lttng-sessiond`**: Kill any existing daemon and restart. Stale sessions from previous crashed runs can prevent event capture.
3. **Source ROS 2 workspace**: `source install/setup.bash`.
4. **Regenerate configs**: Delete old `configs/` directory, run `generate_configs.py`, patch `MAPS_DIR` placeholders.
5. **Clean old output**: Remove `traces/` and `results/` directories.

### 6.2 Per-Configuration Run Loop

For each (block_size, mode, thread_mode, run_number):

| Step | Action | Purpose |
|------|--------|---------|
| 1 | `lttng create interference_exp --output=traces/{run_name}/` | Create a fresh session with per-run output directory |
| 2 | `lttng enable-event --userspace anytime:anytime_compute_entry` | Enable compute block start tracepoint |
| | `lttng enable-event --userspace anytime:anytime_compute_exit` | Enable compute block end tracepoint |
| | `lttng enable-event --userspace anytime:interference_timer_init` | Enable timer init tracepoint |
| | `lttng enable-event --userspace anytime:interference_timer_callback_entry` | Enable timer callback start tracepoint |
| | `lttng enable-event --userspace anytime:interference_timer_callback_exit` | Enable timer callback end tracepoint |
| 3 | `lttng add-context --userspace --type=vpid,vtid,procname` | Attach process/thread context to every event |
| 4 | `lttng start` | Begin recording |
| 5 | `ros2 launch experiments interference_rrt_star.launch.py ...` (background) | Start experiment nodes |
| 6 | `sleep ${RUN_DURATION}` | Let experiment run for 10 s |
| 7 | `lttng stop` + `sleep 1` | Stop recording **before** killing processes (ensures buffer flush) |
| 8 | `kill` + `pkill -9` of all experiment processes | Clean termination |
| 9 | `lttng destroy interference_exp` | Finalize and close session |
| 10 | Verify trace output directory is non-empty | Sanity check |

### 6.3 Why Tracing Stops Before Process Kill

LTTng buffers trace events in shared memory. If processes are killed before `lttng stop`, buffered events may be lost. The 1-second delay between stop and kill allows the daemon to flush all buffers to disk.

### 6.4 Quick Mode vs. Full Mode

| Parameter | Full | Quick |
|-----------|------|-------|
| Block sizes | 1, 16, 64, 256, 1024, 4096, 16384 | 1, 256, 4096, 16384 |
| Runs per config | 5 | 1 |
| Duration per run | 10 s | 5 s |
| Total time | ~70 min | ~3 min |
| Purpose | Publication results | Rapid verification |

## 7. Tracing Methodology

### 7.1 Instrumentation Tool

**LTTng** (Linux Trace Toolkit: next generation) userspace tracing. Key properties:
- **Near-zero overhead**: Events are written to a per-CPU ring buffer in shared memory using lockless algorithms. No system calls in the fast path.
- **No probe effect**: The act of tracing does not measurably perturb the timing being measured.
- **Nanosecond timestamps**: Each event carries a hardware timestamp counter (TSC) value converted to nanoseconds.

### 7.2 Tracepoints Enabled

| Tracepoint | Source | Fields | Role |
|------------|--------|--------|------|
| `anytime:interference_timer_callback_entry` | `interference_timer_node.cpp:46` | `node_handle`, `execution_count` | Primary: marks timer callback start |
| `anytime:interference_timer_callback_exit` | `interference_timer_node.cpp:71` | `node_handle`, `execution_count`, `actual_duration_ns` | Primary: marks timer callback end with measured busy-wait duration |
| `anytime:interference_timer_init` | `interference_timer_node.cpp:22` | `node_handle`, `timer_period_ms`, `execution_time_ms` | Diagnostic: confirms timer configuration |
| `anytime:anytime_compute_entry` | `anytime_base.hpp:164` | `node_handle`, `block_size` | Secondary + warm-up cutoff: marks block start |
| `anytime:anytime_compute_exit` | `anytime_base.hpp:226` | `node_handle`, `iterations_completed`, `computation_time_ns`, `average_time_ns` | Secondary: marks block end with timing |

### 7.3 Context Fields

Every event automatically carries:
- **vpid**: Virtual PID (container-aware process ID)
- **vtid**: Virtual TID (thread ID)
- **procname**: Process name

These allow post-hoc filtering if needed (e.g., to separate events from different containers).

### 7.4 Trace Parsing

`babeltrace` converts the binary CTF (Common Trace Format) output to text. The evaluation script:
1. Runs `babeltrace <trace_dir>` and captures stdout.
2. Pre-filters lines containing `anytime:` (discards kernel context events).
3. Parses each line with regex:
   - Timestamp: `[HH:MM:SS.nanoseconds]` → total nanoseconds
   - Event name: `anytime:event_name:`
   - Fields: `{ key = value, ... }` with type coercion via `int(value, 0)` / `float(value)` try/except chain (handles negative integers, hex pointers like `0x7fff...`, and float values)

## 8. Warm-Up Filtering

### 8.1 Problem

The first several timer callbacks after node startup include ROS 2 initialization overhead: executor setup, DDS discovery, parameter loading, map file parsing, and initial RRT* tree construction. These inflate early timer periods and pollute metrics:
- `avg_timer_period` biased upward
- `max_abs_jitter` reflecting startup, not interference
- `skipped_firings` including startup-related skips

For a 10-second run with ~100 expected callbacks, 5-10 noisy startup samples represent 5-10% of the data.

### 8.2 Approach: Data-Driven Cutoff

The **first `anytime_compute_entry` timestamp** serves as the warm-up cutoff. All events before this timestamp are discarded before metric computation.

This cutoff is:
- **Precise**: marks when the RRT* server has fully initialized and is actually running.
- **Adaptive**: works regardless of how long startup takes on different hardware.
- **Semantically correct**: we only measure interference during actual planning.

### 8.3 Implementation

In `extract_metrics_from_events()`:
1. Scan events for the first `anytime_compute_entry`.
2. If found, discard all events with earlier timestamps.
3. If not found (server never started computing), retain all events.

After filtering, `prev_timer_entry_time` starts as `None`, so the first post-warmup timer event establishes the baseline without computing a period. No artificial period is introduced at the filtering boundary.

## 9. Metrics Computation

**Script**: `evaluate_interference.py`

### 9.1 Per-Run Metrics (Primary: Timer Jitter)

For each trace directory, events are processed sequentially:

| Metric | How computed |
|--------|-------------|
| **Timer period** | Time between consecutive `interference_timer_callback_entry` events. Computed as `(current_timestamp - prev_entry_timestamp)` in nanoseconds, reported in ms. |
| **Jitter** | `timer_period - 100.0` ms. Positive = delayed, negative = early. |
| **Max absolute jitter** | `max(|jitter|)` across all timer periods in the run. |
| **Timer execution time** | Duration from `callback_entry` to `callback_exit`. Measures the actual busy-wait duration (should be ~10 ms). |
| **Skipped firings** | For each timer period: `max(0, round(period / 100.0) - 1)`. If the period was ~200 ms, one firing was skipped; ~300 ms = 2 skipped; etc. Based on ROS 2 wall timer catch-up semantics. |
| **Skipped firings %** | `total_skipped / (total_callbacks + total_skipped) × 100`. |

Summary statistics per run: mean, std, min, max, median for timer periods; mean and std for jitter; mean and std for execution time.

### 9.2 Per-Run Metrics (Secondary: Compute Block)

| Metric | How computed |
|--------|-------------|
| **Compute time** | Duration between paired `anytime_compute_entry` and `anytime_compute_exit` events, in ms. |
| **Total compute blockes** | Count of complete entry/exit pairs. |

Summary statistics: mean, std, min, max for compute times.

**Handling unpaired events**: When the server is cancelled mid-block, `compute()` returns
early without emitting `anytime_compute_exit` (see Section 2.2). This produces an orphaned
`anytime_compute_entry` with no matching exit. The evaluation script handles this by
overwriting `current_compute_entry_time` on each new entry — if a stale entry exists from
a cancelled block, the next entry simply replaces it, and the stale timestamp is discarded.
Only complete entry/exit pairs contribute to compute time metrics.

### 9.3 Aggregation Across Runs

Runs are grouped by base config name (stripping `_runN` suffix). For each group:

| Aggregation | Method |
|-------------|--------|
| avg_timer_period | Mean of per-run avg_timer_period |
| std_timer_period | **Std** of per-run avg_timer_period (cross-run variability) |
| min_timer_period | Min across runs |
| max_timer_period | Max across runs |
| avg_jitter | Mean across runs |
| max_abs_jitter | **Mean** of per-run max_abs_jitter values |
| max_abs_jitter_std | **Std** of per-run max_abs_jitter values |
| skipped_firings_percent | Mean across runs |
| all_timer_periods | Pooled from all runs (for distribution plots) |
| avg_compute_time | Mean of per-run avg_compute_time |
| std_compute_time | **Std** of per-run avg_compute_time (cross-run variability) |

All error bars use cross-run standard deviation of the per-run statistic (not within-run
spread). This ensures bar height and error bar describe the same quantity at the same level
of aggregation. Using within-run std as an error bar on a cross-run mean would mix two
different statistics and could produce bars extending below zero.

## 10. Statistical Methodology

### 10.1 Why 5 Runs per Config

With 5 repeated trials:
- The mean is a reasonable estimate of the expected value.
- The standard deviation captures run-to-run variability (OS scheduling noise, DDS timing, cache effects).
- Total experiment time remains manageable (~70 minutes for 14 configs × 5 runs × 10 s + overhead).

### 10.2 Jitter Error Bars (Option C)

The jitter bar chart presents:
- **Bar height**: `mean(per-run max_abs_jitter)` — average worst-case jitter across runs.
- **Error bar**: `std(per-run max_abs_jitter)` — cross-run standard deviation of worst-case jitter.

**Why not the alternatives?**

| Approach | Problem |
|----------|---------|
| **Naive (original)**: `max(all-run max_abs_jitter)` with `mean(per-run std_jitter)` as error bar | Combines a single extreme value with a completely different statistic. A dispersion measure on a sample maximum from an unrelated quantity is meaningless. |
| **Option A**: `mean(jitter)` ± `std(jitter)` | Average jitter tends toward ~0 because early and late timer firings cancel out. Differences between configs would be invisible. |
| **Option B**: `max(max_abs_jitter)`, no error bar | A single outlier dominates. No information about reproducibility. |
| **Option C (chosen)**: `mean(per-run max_abs_jitter)` ± `std(per-run max_abs_jitter)` | Statistically coherent: both bar height and error bar describe the same quantity (worst-case jitter per run). Answers: "how consistent is the worst-case jitter across repeated trials?" |

For single-run experiments (quick mode), `std` returns 0.0 and the error bar disappears, which is correct.

### 10.3 Timer Period Error Bars

The timer period bar chart uses `avg_timer_period` as the bar height and `std_timer_period` (cross-run std of per-run `avg_timer_period`) as the error bar. This shows "average period ± run-to-run variability."

### 10.4 Compute Time Error Bars

Same pattern: `avg_compute_time` ± cross-run std of per-run `avg_compute_time`.

## 11. Plots Generated

**Output directory**: `results/plots/`

| File | Y-axis | Error bar | X-axis | Description |
|------|--------|-----------|--------|-------------|
| `timer_period_vs_block_size.pdf` | Avg timer period (ms) | std_timer_period | Block size | Shows how block size inflates the timer period beyond the expected 100 ms. Red dashed line at 100 ms. |
| `jitter_vs_block_size.pdf` | Mean max abs jitter (ms) | max_abs_jitter_std | Block size | Worst-case timing deviation, averaged across runs. |
| `skipped_firings_percentage.pdf` | Skipped firings (%) | None | Block size | Percentage of timer firings the executor never dispatched. |
| `compute_time_vs_block_size.pdf` | Avg compute block time (ms) | std_compute_time | Block size | How long each RRT* block holds the executor. |
| `timer_period_distribution.pdf` | Timer period (ms) | Box plot whiskers | Block size | Raw period distributions. Two panels: reactive (left), proactive (right). Data pooled across all runs per config. |
| `legend.pdf` | — | — | — | Standalone legend for figure composition. |

### Plot Conventions
- X-axis: block sizes sorted **descending** (high to low). This is an intentional choice: the largest block sizes (leftmost) show the most interference, creating a visual "staircase" from disrupted to ideal.
- Bar grouping: reactive and proactive side by side for each block size.
- Style: `seaborn-v0_8-darkgrid`.
- Format: PDF at 300 DPI.

## 12. Output Files

| File | Format | Content |
|------|--------|---------|
| `results/individual_runs.csv` | CSV | One row per trace: config name, block_size, all per-run metrics |
| `results/aggregated_results.csv` | CSV | One row per config: aggregated metrics across runs |
| `results/aggregated_results.json` | JSON | Same as aggregated CSV, nested by config name |
| `results/table_1_skipped_firings.csv` | CSV | Condensed table: skipped firings % by mode (rows) and block size (columns) |
| `results/plots/*.pdf` | PDF | All plots listed above |

## 13. Expected Results and Interpretation

### 13.1 Expected Trends

- **Small block sizes (1, 16, 64)**: Compute time per block << 100 ms. Timer period should be close to 100 ms. Jitter should be small. No skipped firings.
- **Medium block sizes (256, 1024)**: Compute time approaches or crosses 100 ms. Timer period inflates. Some jitter. Occasional skipped firings.
- **Large block sizes (4096, 16384)**: Compute time >> 100 ms. Timer periods are multiples of the compute time. Large jitter. Significant skipped firings.

### 13.2 Reactive vs. Proactive Difference

The difference should be subtle. Proactive mode calculates the result (path cost, tree statistics) on every cycle, adding overhead between blocks. Reactive mode only calculates the result when finishing or cancelling. In practice, this difference may be negligible compared to the block computation itself.

### 13.3 What Validates the Experiment

1. Timer period for block_size=1 should be close to 100 ms (near-zero interference).
2. Timer period should monotonically increase with block size.
3. Compute block time should scale proportionally with block size.
4. Skipped firings should appear only when compute time exceeds 100 ms.

## 14. Reproducibility

### 14.1 Deterministic Factors
- `random_seed: 42` ensures the same RRT* tree is constructed across runs.
- Fixed start/goal positions eliminate path variation.
- YAML configs are regenerated from scratch (idempotent).

### 14.2 Non-Deterministic Factors
- OS scheduling: other system processes may introduce timer jitter.
- DDS discovery timing: initial node handshake varies.
- Cache state: CPU cache warming differs across runs.
- These are captured by the 5-run averaging and error bars.

### 14.3 Test Scripts
- `test_single_config.sh`: Runs a single config (block_256_reactive_single) for 10 s and verifies tracepoints are captured. Use this to validate the setup before running the full experiment.
- `run_quick.sh`: Runs a reduced parameter sweep (4 block sizes, 1 run each, 5 s) to verify the entire pipeline in ~3 minutes.

## 15. Design Decisions Log

| # | Decision | Rationale |
|---|----------|-----------|
| 1 | Interference timer parameters loaded from YAML config file (not inline launch args) | Consistency with server/client config handling; all parameters traceable to versioned YAML files |
| 2 | Data-driven warm-up cutoff (first `anytime_compute_entry`) | Adaptive to actual startup duration; semantically correct; avoids arbitrary time window |
| 3 | Option C for jitter error bars: mean(per-run max) ± std(per-run max) | Statistically valid: bar height and error bar describe the same quantity |
| 4 | Single-threaded executor as default in launch file | Matches experiment scope; prevents accidental multi-threaded runs during manual debugging |
| 5 | Descending x-axis on all bar plots | Intentional: largest block sizes (most interference) appear first, creating a visual staircase to ideal timing |
| 6 | 5 runs × 10 s per config | Balances statistical power with total experiment duration (~70 min) |
| 7 | Client in separate container | Isolates client timer scheduling from server/interference measurement. Client timers do not compete for the server executor. |
| 8 | Busy-wait (not sleep) for interference timer | `sleep` would yield the thread; busy-wait holds the executor for a known, measurable duration. This faithfully simulates a compute-bound periodic task. |
| 9 | Wall timer (not ROS timer) for interference | Wall timers use wall-clock time, matching real-time expectations. ROS timers would depend on `/clock` topic, which is not published in this experiment. |
| 10 | 100,000 iteration goal from client | Intentionally unreachable within the 200 ms cancel window. Ensures the server is always actively computing when cancelled, never idle. |
| 11 | Discard orphaned `compute_entry` events (no matching `exit`) | Cancellation mid-block causes `compute()` to return without emitting `exit`. Overwriting the entry timestamp on the next entry prevents stale/inflated compute time measurements. |
| 12 | Config YAMLs contain only parameters read by nodes | Removed unused `multi_threading` (threading is set by executor type in launch file) and `log_level` (set via `--ros-args --log-level` on the container). Prevents confusion about what actually controls behavior. |
| 13 | Trace field parsing uses `int(value, 0)` / `float(value)` try/except | Handles negative integers, hex pointers (`0x...`), and edge cases that `str.isdigit()` misses. |
