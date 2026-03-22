# RRT* Experiment Methodology

This document walks through every step of the RRT* experiment — from configuration generation through data collection to evaluation — explaining what happens, why, and what to watch for when interpreting results.

---

## 1. Objective

The experiment measures the **Anytime framework's** scheduling overhead, cancellation responsiveness, and feedback throughput using RRT* path planning as the domain algorithm. The independent variables are batch size, cancellation mode (reactive vs proactive), threading model (single vs multi), and map environment. The dependent variables are batch compute time, overhead ratio, cancellation latency, and solution quality.

This is a **framework evaluation**, not an algorithm benchmark. RRT* is the workload, not the subject.

---

## 2. Configuration Generation

**Script:** `generate_configs.py`

### Independent Variables

| Variable | Values | Count | Rationale |
|----------|--------|-------|-----------|
| Batch size | 1, 16, 64, 256, 1024, 4096 | 6 | Spans single-iteration (isolates per-iteration framework overhead) to large batches (approaches continuous computation). Roughly powers-of-4 progression. |
| Cancellation mode | reactive, proactive | 2 | The two scheduling strategies the framework provides. |
| Threading model | single, multi | 2 | Tests whether multi-threaded executors affect overhead or responsiveness. |
| Map | depot, warehouse | 2 | Structurally different environments to check that framework behavior is algorithm-independent. |

**Total:** 6 x 2 x 2 x 2 = **96 unique configurations**

### Fixed Parameters

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Run duration | 10 s | Long enough for ~20 goal cycles at 500 ms period, producing hundreds to thousands of batch measurements per run. |
| Runs per configuration | 5 | Captures run-to-run variability from OS scheduling, RNG divergence, and timer jitter. |
| Goal timer period | 500 ms | Client sends a new planning goal every 500 ms. Sets the goal cycle period. |
| Cancel timeout | 200 ms | Client cancels 200 ms after sending goal. Gives the planner ~200 ms of compute per cycle. |
| Goal iteration target | 100,000 | Set high enough that the planner never finishes naturally; all terminations are via cancellation. |
| Random seed | 42 | Reproducible RNG initialization for the RRT* sampler. |
| Step size | 0.5 m | Maximum extension distance per RRT* iteration. Also used as upper bound for the near-radius in rewiring. |
| Goal threshold | 0.5 m | A tree node within this distance of the goal counts as a solution. |
| Goal bias | 0.05 | 5% probability of sampling the goal point directly, accelerating solution discovery. |
| Gamma (RRT*) | 0.0 (auto) | Auto-computed from map free area: `gamma = 2 * sqrt(1.5 * free_area / pi)`. |
| Prune interval | 1000 | Branch-and-bound pruning every 1000 iterations (only when a solution exists). |
| Convergence log interval | 100 | Emit an `rrt_star_iteration` tracepoint every 100 iterations. |

### Config File Generation

For each of the 96 configurations, `generate_configs.py` produces two YAML files:

- **Server config** (`batch_X_mode_threading_map_server.yaml`): Sets `is_reactive_proactive`, `multi_threading`, `batch_size`, `random_seed`, map path (placeholder `MAPS_DIR`), start/goal coordinates, and all RRT* algorithm parameters.
- **Client config** (`batch_X_mode_threading_map_client.yaml`): Sets `goal_timer_period_ms` and `cancel_timeout_period_ms`.

The `MAPS_DIR` placeholder is replaced at runtime by the run script with the actual installed map path via `sed`.

### Map Environments

| Map | Dimensions | Resolution | World Size | Start | Goal | Character |
|-----|-----------|------------|------------|-------|------|-----------|
| Depot | 604 x 307 px | 0.05 m/px | 30.2 x 15.4 m | (5.0, 12.0) | (25.0, 2.0) | Small, mostly open with scattered obstacles |
| Warehouse | 1006 x 1674 px | 0.03 m/px | 30.2 x 50.2 m | (-12.5, -20.0) | (12.5, 20.0) | Large, structured with narrow aisles |

Maps are Nav2-format PGM occupancy grids with YAML metadata. Collision checking uses grid-based ray-tracing at half-resolution sampling intervals along edges.

---

## 3. Experiment Execution

**Script:** `run_rrt_star_experiments.sh`

### 3.1 Pre-Flight

1. Verify `lttng` is installed.
2. Kill any existing `lttng-sessiond`, start a fresh daemon, wait 3 seconds for stabilization.
3. Source the ROS 2 workspace (`source install/setup.bash`).
4. Resolve the installed map directory path via `ros2 pkg prefix anytime_rrt_star`.
5. **Regenerate configs** by calling `generate_configs.py` (ensures configs are never stale after parameter changes), then substitute the `MAPS_DIR` placeholder with the actual path.
6. Clean old traces and results directories.

### 3.2 Per-Run Execution (repeated 480 times)

For each (batch_size, mode, threading, map, run_number):

#### Step 1: LTTng Session Setup

- Create a named LTTng session (`rrt_star_exp`) with output to `traces/batch_X_mode_threading_map_runN/`.

#### Step 2: Enable Tracepoints

Three categories of userspace tracepoints are enabled:

**Framework core** (7 events):
- `anytime_compute_entry` / `anytime_compute_exit` — batch computation boundaries
- `anytime_server_handle_cancel` — server receives cancel request
- `anytime_base_deactivate` — server finishes processing cancel/result
- `anytime_client_goal_sent` / `anytime_client_cancel_sent` / `anytime_client_goal_finished` — client lifecycle timestamps

**Overhead measurement** (4 events):
- `anytime_send_feedback_entry` / `anytime_send_feedback_exit` — feedback publish overhead
- `anytime_calculate_result_entry` / `anytime_calculate_result_exit` — result computation overhead

**RRT*-specific** (4 events):
- `rrt_star_iteration` — periodic algorithm state (every 100 iterations)
- `rrt_star_result` — final metrics at cancellation
- `rrt_star_init` — algorithm initialization
- `rrt_star_reset` — tree cleared for new goal cycle

**Context fields:** vpid, vtid, procname (for thread/process identification).

#### Step 3: Start Tracing and Launch Experiment

- `lttng start` begins event collection.
- Launch the ROS 2 experiment via `ros2 launch experiments rrt_star.launch.py` with the appropriate server/client config and threading mode.
- **Sleep for 10 seconds** — this is the measurement window.

#### Step 4: Stop Tracing and Tear Down

The order is critical:

1. `lttng stop` — **flushes all buffered events to disk before killing processes.** This ensures no trace data is lost.
2. Sleep 1 second for flush completion.
3. `kill $LAUNCH_PID` (SIGTERM for graceful shutdown).
4. Sleep 1 second.
5. `kill -9` and `pkill -9` for any orphaned processes.
6. `lttng destroy` to clean up the session.
7. Verify trace output exists.
8. Sleep 1 second between runs for system cooldown.

### 3.3 Node Architecture

The launch file (`rrt_star.launch.py`) sets up the experiment differently depending on the threading model:

**Multi-threaded mode:**
- A single `component_container_mt` (multi-threaded executor) hosts both the action server and client as composable nodes.
- Multiple threads service callbacks; the compute callback group is MutuallyExclusive so only one thread runs the compute loop at a time, but other threads can process cancel requests and timer callbacks concurrently.

**Single-threaded mode:**
- Two separate `component_container` processes: one for the server, one for the client.
- Each runs a single-threaded executor, so all callbacks (compute, cancel, timers) execute sequentially within their process.

This difference is fundamental: in multi-threaded mode, a cancel request can be processed on a different thread while compute is running (the compute loop checks `goal_handle_->is_canceling()` between iterations). In single-threaded mode, the cancel callback can only run after the current compute batch completes.

---

## 4. Execution Model

### 4.1 Client State Machine

The client operates on a repeating goal cycle:

1. **Goal timer fires** (every 500 ms): Client sends a goal with target = 100,000 iterations and disables the goal timer.
2. **Goal accepted**: Client starts the cancel timeout timer (200 ms).
3. **Cancel timeout fires** (200 ms after acceptance): Client sends an async cancel request.
4. **Result received** (shortly after cancel): Client logs result, resets state, re-enables the goal timer.
5. **Next goal timer fires** (500 ms after original goal): Cycle repeats.

The goal iteration target (100,000) is deliberately unreachable in the ~200 ms compute window, ensuring all terminations happen via cancellation, not natural completion. This isolates the framework's cancel-handling path.

### 4.2 Server Execution Flow

On receiving a goal:

1. `handle_goal()` — Accept if not already executing (mutual exclusion: one goal at a time).
2. `handle_accepted()` — Set goal handle, call `reset()` (clears domain state), call `activate()` (sets `is_running_ = true`), call `start()` (triggers the waitable).
3. **Waitable executes** the mode-specific function:

**Reactive mode** (`reactive_anytime_function`):
```
compute() → check finish/cancel → if continuing: send_feedback() → reschedule
                                 → if done: calculate_result() → succeed/cancel → deactivate
```

**Proactive mode** (`proactive_anytime_function`):
```
compute() → check finish/cancel → if continuing: calculate_result() → send_feedback() → reschedule
                                 → if done: succeed/cancel (with cached result) → deactivate
```

The key difference: in proactive mode, `calculate_result()` runs every iteration (between compute and feedback), so the result is always ready. In reactive mode, it runs only at termination, reducing per-batch overhead but delivering a potentially stale result.

### 4.3 Compute Batch

Each `compute()` call executes `batch_size` iterations of `compute_single_iteration()`:

```cpp
TRACE_ANYTIME_COMPUTE_ENTRY(node_, iterations);
auto start_time = node_->now();

for (int i = 0; i < iterations; i++) {
    if (goal_handle_->is_canceling() || !goal_handle_->is_executing() || !is_running()) {
        return;  // early exit, no COMPUTE_EXIT trace
    }
    compute_single_iteration();
}

// ... timing, averaging ...
TRACE_ANYTIME_COMPUTE_EXIT(node_, iterations, computation_time_ns, avg_time_ns);
```

**Cancellation is per-iteration, not per-batch.** Even with batch_size=4096, the cancel check runs before every single RRT* iteration. Cancel response delay is bounded by one iteration's time (microseconds to low milliseconds), regardless of batch size. The `batch_size` parameter controls **overhead-to-compute ratio** and **feedback frequency**, not cancellation responsiveness.

**Missing exit trace on cancellation:** When cancellation occurs mid-batch, the `return` at the check skips `TRACE_ANYTIME_COMPUTE_EXIT`. This creates an unmatched `compute_entry` event. The evaluation script handles this by overwriting `current_compute_start` on the next entry — the interrupted batch produces no `batch_time` measurement. This affects the last batch of every goal cycle equally across all configurations.

### 4.4 RRT* Algorithm per Iteration

Each `compute_single_iteration()` runs one full RRT* iteration:

1. **Sample** — Uniform random point in map bounds, with 5% goal bias.
2. **Find nearest** — Linear scan (O(n)) over all tree nodes.
3. **Steer** — Move up to `step_size` (0.5 m) toward the sample.
4. **Collision check** — Ray-trace along the edge at half-resolution intervals.
5. **Find near** — All nodes within `gamma * sqrt(log(n)/n)` radius (O(n) scan, radius shrinks as tree grows).
6. **Choose parent** — Among near nodes, pick the one giving lowest cost-from-start (with collision checks).
7. **Insert node** — Add to tree with chosen parent.
8. **Rewire** — For each near node, check if routing through the new node reduces cost; if so, rewire and propagate cost updates to children (recursive).
9. **Goal check** — If within `goal_threshold` (0.5 m) of goal and cost improves, update `best_path_cost_`.
10. **Prune** — Every `prune_interval` (1000) iterations, branch-and-bound prune: remove nodes that cannot improve the current best solution.

After the iteration, `loop_count_` is incremented. Every `convergence_log_interval` (100) iterations, an `rrt_star_iteration` tracepoint is emitted with `(loop_count_, tree_size, best_path_cost_)`.

### 4.5 Goal Cycling and Tree Resets

Each new goal triggers `reset_domain_state()`, which:

- Clears the entire tree
- Resets `loop_count_` to 0
- Resets `best_path_cost_` to infinity
- Re-adds the start node

With 500 ms goal period and ~200 ms cancel timeout, a 10-second run produces approximately **20 goal cycles**. Each cycle is an independent planning episode starting from scratch. The `best_cost_at_cancellation` in results reflects ~200 ms of planning, not asymptotic RRT* convergence.

### 4.6 Seed and Reproducibility

The RNG (`std::mt19937`) is initialized once with `seed=42` at construction. It is **not** re-seeded in `reset_domain_state()`. Consequences:

- The first goal cycle of every run starts with identical RNG state (for the same map).
- After the first cycle, RNG state depends on how many iterations were completed, which depends on OS scheduling jitter and exact cancellation timing.
- Across repeated runs, RNG trajectories diverge after the first cycle.
- The 5 runs per configuration are designed to capture this timing-dependent variability.

---

## 5. Tracing and Data Collection

### 5.1 Instrumentation

All tracepoints use LTTng userspace tracing (UST). Events are written to kernel ring buffers and flushed to CTF (Common Trace Format) files on disk when `lttng stop` is called.

The tracing overhead itself is minimal (nanosecond-scale per event) and does not significantly affect measurements. However, the `rrt_star_iteration` tracepoint is emitted only every 100 iterations to avoid trace buffer saturation at high iteration rates.

### 5.2 Trace Output

Each run produces a CTF trace directory: `traces/batch_X_mode_threading_map_runN/`. These are read by `babeltrace` during evaluation.

### 5.3 What the Traces Capture

| Measurement | How It's Computed | Events Used |
|-------------|-------------------|-------------|
| Batch compute time | `compute_exit.timestamp - compute_entry.timestamp` | compute_entry, compute_exit |
| Inter-batch overhead | `compute_entry[N+1].timestamp - compute_exit[N].timestamp` (excluding inter-cycle gaps) | compute_entry, compute_exit, rrt_star_reset, base_deactivate |
| Overhead ratio | `overhead / (overhead + batch_time) * 100%` | compute_entry, compute_exit |
| Server cancel response | `base_deactivate.timestamp - server_handle_cancel.timestamp` | server_handle_cancel, base_deactivate |
| Goal-to-finish latency | `goal_finished.timestamp - goal_sent.timestamp` | client_goal_sent, client_goal_finished |
| Goal-to-cancel latency | `cancel_sent.timestamp - goal_sent.timestamp` | client_goal_sent, client_cancel_sent |
| Cancel-to-finish latency | `goal_finished.timestamp - cancel_sent.timestamp` | client_cancel_sent, client_goal_finished |
| Feedback send time | `feedback_exit.timestamp - feedback_entry.timestamp` | send_feedback_entry, send_feedback_exit |
| Result compute time | `result_exit.timestamp - result_entry.timestamp` | calculate_result_entry, calculate_result_exit |
| Convergence data | `(iteration_num, best_cost, tree_size)` | rrt_star_iteration, rrt_star_result |
| Cycle boundaries | New cycle detected | rrt_star_reset |

---

## 6. Evaluation

**Script:** `evaluate_rrt_star.py`

### 6.1 Trace Parsing

1. Discover all trace directories matching `traces/batch_*`.
2. For each directory, invoke `babeltrace` to decode CTF events.
3. Parse each line: extract timestamp (converted to nanoseconds), event name, and field values.
4. Filter for events in the `anytime:` namespace.

### 6.2 Per-Run Metric Extraction

For each run's trace events, the script maintains state machines to pair entry/exit events and compute deltas:

- **Batch times**: Paired compute_entry/exit timestamps (ms).
- **Per-batch overheads**: Gap between consecutive compute_exit and compute_entry (ms). Gaps that span a goal cycle boundary (detected via `rrt_star_reset` or `base_deactivate` events) are excluded — these represent inter-goal idle time (~300 ms), not framework overhead.
- **Overhead ratios**: Computed at each compute_entry using the previous batch's time as the denominator: `overhead / (overhead + prev_batch_time) * 100`. Only measured for within-cycle gaps.
- **Cancel response delays**: Paired server_handle_cancel to base_deactivate (ms).
- **Client latencies**: goal_sent to goal_finished, goal_sent to cancel_sent, cancel_sent to goal_finished (ms).
- **Feedback/result times**: Paired entry/exit for send_feedback and calculate_result (ms).
- **Convergence data**: Stored as a list of goal cycles (detected via `rrt_star_reset` events), each cycle containing a list of `(iteration, best_cost, tree_size)` tuples from `rrt_star_iteration` and `rrt_star_result` events.
- **Final metrics (best cost, tree size, total iterations)**: Only the *last* `rrt_star_result` event per goal cycle is used. In proactive mode, `rrt_star_result` fires every batch (via `calculate_result()`); using only the last one per cycle avoids inflating `total_iterations` (which would otherwise sum intermediate iteration counts) and prevents early `inf` costs from poisoning `avg_final_best_cost`.

Summary statistics per run: mean, std, percentiles (p50, p95, p99) for batch times; mean and std for all other metrics.

### 6.3 Aggregation Across Runs

Runs sharing the same base configuration (ignoring `_runN` suffix) are grouped and aggregated:

- **Means**: Simple average of per-run means.
- **Standard deviations**: Combined using the **law of total variance**:
  ```
  sigma_combined = sqrt(E[sigma_i^2] + Var(mu_i))
  ```
  This accounts for both within-run variability (average of variances) and between-run variability (variance of means). A simple average of standard deviations would systematically underestimate true variability.
- **Percentiles**: Averaged across runs (note: this is an approximation; computing percentiles from pooled data would be more precise but requires storing all raw values).
- **Filtered fields**: Some metrics (cancel response, latencies) filter out zero-valued runs before aggregating.

### 6.4 Statistical Significance Tests

Mann-Whitney U tests (two-sided, from scipy) compare:

- **Reactive vs proactive**: For each (batch_size, map, threading) triple, pooling raw batch_times across runs for each mode.
- **Single vs multi threading**: For each (batch_size, map, mode) triple, pooling raw batch_times for each threading model.

Mann-Whitney U is chosen over the t-test because batch time distributions may be non-normal (especially at small batch sizes where OS scheduling noise dominates). The minimum sample size is 5 per group. Results are saved to `results/statistical_tests.csv` with U statistic, p-value, and a significance flag at alpha=0.05.

The scipy dependency is optional — the script continues without tests if scipy is not installed.

### 6.5 Convergence Curve Averaging

Since `loop_count_` resets at each goal cycle, raw convergence data consists of multiple short segments. The evaluation script:

1. Groups all goal cycles by configuration (batch_size, mode, threading) across all 5 runs (~100 cycles per config).
2. Collects all unique iteration numbers where a finite cost was observed.
3. For each iteration point, forward-fills from each cycle: uses the last known cost at or before that iteration.
4. Computes mean and standard deviation across all cycles at each iteration.
5. Plots the mean line with standard deviation shading.

This produces a single averaged convergence curve per configuration, rather than plotting up to 100 individual cycles (which would be unreadable).

### 6.6 Generated Outputs

**CSV files:**

| File | Contents |
|------|----------|
| `results/individual_runs.csv` | All per-run metrics (one row per run, 480 rows) |
| `results/aggregated_results.csv` | Aggregated per-config metrics with parsed dimensions (one row per config, 96 rows) |
| `results/aggregated_results.json` | Same aggregated data in JSON format |
| `results/convergence_data/{map}_convergence.csv` | Raw convergence data with config, cycle, iteration, best_cost, tree_size columns |
| `results/statistical_tests.csv` | Mann-Whitney U test results for all comparisons |

**Plots (PDF):**

Framework metrics (`results/plots/framework/`):
- `batch_size_vs_time[_{map}].pdf` — Batch compute time vs batch size, grouped by mode x threading
- `throughput[_{map}].pdf` — Iterations per second vs batch size
- `server_cancel_response[_{map}].pdf` — Cancel handling latency
- `goal_to_finish_latency.pdf` — End-to-end goal lifecycle time
- `goal_to_cancel_latency.pdf` — Time from goal send to cancel send
- `cancellation_latency.pdf` — Time from cancel send to result receive
- `total_iterations.pdf` — Iterations completed per configuration
- `total_cancellation_time.pdf` — Combined cancel and response time

Overhead metrics (`results/plots/overhead/`):
- `per_batch_overhead[_{map}].pdf` — Inter-batch gap time
- `overhead_ratio[_{map}].pdf` — Overhead as percentage of total cycle time
- `feedback_send_time[_{map}].pdf` — Feedback publish latency
- `result_compute_time[_{map}].pdf` — Result calculation time
- `batch_time_p{50,95,99}[_{map}].pdf` — Batch time percentiles, grouped by mode x threading
- `batch_time_trend[_{map}].pdf` — Batch time over sequential batches (aggregated by config, mean with std shading)

RRT*-specific metrics (`results/plots/rrt_star/`):
- `convergence_curve_{map}.pdf` — Averaged convergence curves with std shading per config
- `convergence_by_map.pdf` — Convergence comparison between maps (averaged)
- `first_solution_iteration_{map}.pdf` — First iteration finding a solution, per config
- `best_cost_vs_batch_size[_{map}].pdf` — Solution quality at cancellation
- `tree_size_vs_iterations_{map}.pdf` — Averaged tree growth curves with std shading

Other:
- `results/plots/legend.pdf` — Standalone legend for bar charts (4 groups: reactive-single, reactive-multi, proactive-single, proactive-multi)
- `results/plots/maps/{map}.{png,pdf}` — Map visualizations with start/goal markers

All grouped bar charts use **explicit color assignment** (`C0`=reactive-single, `C1`=reactive-multi, `C2`=proactive-single, `C3`=proactive-multi) to prevent color-legend mismatches when some groups have no data for a given map. All bar charts include inline legends.

---

## 7. Interpreting Results

### What Batch Size Controls

Batch size does **not** control cancellation responsiveness (which is per-iteration). It controls:

- **Overhead-to-compute ratio**: At batch_size=1, the framework overhead path (feedback send, timer checks, waitable rescheduling) runs after every single iteration. At batch_size=4096, it runs once per 4096 iterations. The overhead ratio should decrease with larger batch sizes.
- **Feedback frequency**: The client receives feedback (current best cost, tree size) once per batch. Smaller batches = more frequent feedback.
- **Convergence tracepoint resolution**: With convergence_log_interval=100, a batch of 1 emits a tracepoint every 100 batches, while a batch of 4096 emits one every ~0.024 batches.

### What the Convergence Plots Show

Not asymptotic RRT* convergence. Each goal cycle gets ~200 ms of compute. The plots show how quickly cost decreases within these short planning windows. Different batch sizes may show different convergence depths because the overhead ratio affects how many iterations fit into the 200 ms window.

### Cost Metric

`best_path_cost_` records the cost-from-start to the nearest tree node within `goal_threshold` (0.5 m) of the goal. It does **not** include the final segment to the exact goal point. This underestimates true path cost by up to 0.5 m (~1-2% for typical paths). The bias is constant across configurations.

### Cross-Map Comparisons

The depot and warehouse maps have very different characteristics (open vs. constrained, small vs. large). Results should primarily be compared within the same map. Cross-map differences reflect algorithm behavior, not framework behavior.

---

## 8. Known Limitations

1. **Missing `TRACE_ANYTIME_COMPUTE_EXIT` on mid-batch cancellation.** The interrupted batch produces no timing measurement. `total_batches` (incremented at entry) can exceed `len(batch_times)` (recorded at exit). Affects the last batch of every goal cycle equally across configurations.

2. **Batch time non-stationarity.** `find_nearest()` and `find_near()` are O(n) linear scans. Per-iteration cost grows as the tree grows within each cycle. `avg_time_per_batch` mixes framework overhead variability with algorithmic complexity growth. The batch time trend plots expose this; for pure framework analysis, restrict to early batches or normalize by tree size.

3. **Percentile aggregation is approximate.** Per-run percentiles are averaged across runs rather than computed from pooled data. For most practical purposes the difference is negligible with 5 runs.

4. **Convergence subsampling.** The `rrt_star_iteration` tracepoint fires every 100 iterations. At large batch sizes, this means very few tracepoints per batch, giving coarse convergence resolution. At batch_size=4096, a single batch contains ~41 tracepoint emissions (4096/100), but if a goal cycle only lasts a few batches, total data points per cycle are sparse.

5. **RNG reproducibility is limited.** The seed is set once; RNG state diverges across goal cycles due to OS timing. The 5 repeated runs capture this variability, but individual cycles within a run are not independently reproducible.

6. **Collision checking cost.** Ray-tracing along edges at half-resolution intervals adds O(edge_length / resolution) cost per collision check. The warehouse map (resolution=0.03 m) has ~3x more samples per edge than the depot (resolution=0.05 m), making per-iteration cost higher on the warehouse map independent of tree size.

7. **Single action goal at a time.** The server rejects overlapping goals (`handle_goal` checks `is_running()`). If the client sends a goal before the previous cycle completes, it is rejected and retried on the next timer tick. This should be rare given the 500 ms goal period and ~200 ms cycle time.

8. **Proactive mode sends stale result on first-call termination.** In the proactive execution path (`anytime_base.hpp:125-131`), if cancellation occurs during the very first compute batch, `result_` is sent without ever calling `calculate_result()` — the client receives a default-initialized (zeroed) result. Reactive mode does not have this issue (it always calls `calculate_result()` before sending). In practice this is extremely rare: the cancel would need to arrive within microseconds of the first compute starting, and the result is still correctly reported in subsequent cycles.

9. **Proactive mode emits many more convergence data points.** In proactive mode, `calculate_result()` runs every batch, emitting an `rrt_star_result` tracepoint each time. For batch_size=1, this means one result event per iteration (~22,000 per run) vs. one per goal cycle (~6 per run) in reactive mode. The evaluation script handles this correctly — only the last result per cycle is used for `final_best_costs` and `total_iterations` — but the raw convergence data volume is ~100x larger in proactive mode.

10. **Overhead ratio uses previous batch time as denominator.** The overhead ratio is computed as `overhead / (overhead + prev_batch_time)`, pairing each overhead with the batch it *follows*. An alternative would pair overhead with the batch it *precedes*. Both are valid; readers should be aware of the convention.

11. **Recursive cost propagation in rewire.** `propagate_cost_update()` in `anytime_management.hpp:382-390` uses recursive descent to update child costs after rewiring. On pathologically deep tree chains this could overflow the call stack. In practice, with ~200 ms planning windows and step_size=0.5 m, tree depths stay well within safe limits.
