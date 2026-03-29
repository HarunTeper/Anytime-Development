# RRT* Evaluation Plan

## 1. Overview

This document describes how to evaluate the `anytime_rrt_star` package using the same infrastructure as the Monte Carlo experiments: LTTng tracing, parameter sweeps, Docker containers, and automated plotting. The evaluation answers two questions:

1. **Framework metrics** (same as Monte Carlo): How do block size, mode, and threading affect throughput, cancellation delay, and latency?
2. **RRT*-specific metrics** (new): How does path cost converge over time? How does the anytime property manifest across configurations?

## 2. What Needs to Be Built

### 2.1 Tracing Infrastructure (anytime_tracing package)

**Current state:** RRT* tracing macros in `tracing.hpp` are placeholders (empty `#define`s). We need to register real LTTng tracepoints.

**Files to modify:**

1. **`anytime_tracing/include/anytime_tracing/tp_call.h`** — Add RRT* tracepoint events:

```c
// ==================== RRT* ====================

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, rrt_star_init,
  TP_ARGS(
    const void *, node_handle_arg,
    const int, block_size_arg,
    const bool, is_reactive_proactive_arg),
  TP_FIELDS(
    ctf_integer_hex(const void *, node_handle, node_handle_arg)
    ctf_integer(int, block_size, block_size_arg)
    ctf_integer(bool, is_reactive_proactive, is_reactive_proactive_arg)
    ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, rrt_star_iteration,
  TP_ARGS(
    const void *, node_handle_arg,
    const int, iteration_num_arg,
    const int, tree_size_arg,
    const double, best_cost_arg),
  TP_FIELDS(
    ctf_integer_hex(const void *, node_handle, node_handle_arg)
    ctf_integer(int, iteration_num, iteration_num_arg)
    ctf_integer(int, tree_size, tree_size_arg)
    ctf_float(double, best_cost, best_cost_arg)
    ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, rrt_star_result,
  TP_ARGS(
    const void *, node_handle_arg,
    const double, best_cost_arg,
    const int, total_iterations_arg,
    const int, tree_size_arg),
  TP_FIELDS(
    ctf_integer_hex(const void *, node_handle, node_handle_arg)
    ctf_float(double, best_cost, best_cost_arg)
    ctf_integer(int, total_iterations, total_iterations_arg)
    ctf_integer(int, tree_size, tree_size_arg)
    ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, rrt_star_reset,
  TP_ARGS(const void *, node_handle_arg),
  TP_FIELDS(
    ctf_integer_hex(const void *, node_handle, node_handle_arg)
    ctf_string(version, anytime_tracing_VERSION)))
```

2. **`anytime_tracing/include/anytime_tracing/anytime_tracetools.h`** — Add declarations:

```c
ANYTIME_DECLARE_TRACEPOINT(rrt_star_init, const void *, const int, const bool)
ANYTIME_DECLARE_TRACEPOINT(rrt_star_iteration, const void *, const int, const int, const double)
ANYTIME_DECLARE_TRACEPOINT(rrt_star_result, const void *, const double, const int, const int)
ANYTIME_DECLARE_TRACEPOINT(rrt_star_reset, const void *)
```

3. **`anytime_tracing/src/anytime_tracetools.c`** — Add wrapper implementations:

```c
// ==================== RRT* ====================

void ANYTIME_TRACEPOINT(
  rrt_star_init, const void * node_handle, const int block_size,
  const bool is_reactive_proactive)
{
  CONDITIONAL_TP(rrt_star_init, node_handle, block_size, is_reactive_proactive);
}

void ANYTIME_TRACEPOINT(
  rrt_star_iteration, const void * node_handle, const int iteration_num,
  const int tree_size, const double best_cost)
{
  CONDITIONAL_TP(rrt_star_iteration, node_handle, iteration_num, tree_size, best_cost);
}

void ANYTIME_TRACEPOINT(
  rrt_star_result, const void * node_handle, const double best_cost,
  const int total_iterations, const int tree_size)
{
  CONDITIONAL_TP(rrt_star_result, node_handle, best_cost, total_iterations, tree_size);
}

void ANYTIME_TRACEPOINT(rrt_star_reset, const void * node_handle)
{
  CONDITIONAL_TP(rrt_star_reset, node_handle);
}
```

4. **`anytime_rrt_star/include/anytime_rrt_star/tracing.hpp`** — Replace placeholders with real macros:

```cpp
#define TRACE_RRT_STAR_INIT(node, block_size, is_reactive_proactive) \
  ANYTIME_TRACEPOINT( \
    rrt_star_init, static_cast<const void *>(node->get_node_base_interface().get()), \
    block_size, is_reactive_proactive)

#define TRACE_RRT_STAR_ITERATION(node, iteration_num, tree_size, best_cost) \
  ANYTIME_TRACEPOINT( \
    rrt_star_iteration, static_cast<const void *>(node->get_node_base_interface().get()), \
    iteration_num, tree_size, best_cost)

#define TRACE_RRT_STAR_RESULT(node, best_cost, total_iterations, tree_size) \
  ANYTIME_TRACEPOINT( \
    rrt_star_result, static_cast<const void *>(node->get_node_base_interface().get()), \
    best_cost, total_iterations, tree_size)

#define TRACE_RRT_STAR_RESET(node) \
  ANYTIME_TRACEPOINT( \
    rrt_star_reset, static_cast<const void *>(node->get_node_base_interface().get()))
```

**Note on `rrt_star_iteration`:** This tracepoint fires once per iteration and carries `best_cost`. This is the key data source for convergence curves. However, enabling it for every iteration generates significant trace volume. For framework metrics (throughput, latency), we only need the core `anytime:anytime_compute_entry/exit` and client tracepoints — same as Monte Carlo. The `rrt_star_iteration` tracepoint should be **optionally enabled** when convergence data is needed.

### 2.2 Action Interface Enhancement

**Current `RrtStar.action`:**
```
# Goal
int32 goal
---
# Result
float32 result
int32 iterations
int32 block_size
builtin_interfaces/Duration block_time
builtin_interfaces/Time action_server_receive
builtin_interfaces/Time action_server_accept
builtin_interfaces/Time action_server_start
builtin_interfaces/Time action_server_cancel
builtin_interfaces/Time action_server_send_result
---
# Feedback
float32 feedback
```

**Proposed additions to Result:**
```
int32 tree_size                    # final tree size (number of nodes)
int32 first_solution_iteration     # iteration at which first path was found (-1 if none)
```

These two fields add RRT*-specific information to each result without requiring tracepoint changes. The `first_solution_iteration` is particularly useful — it tells us how quickly the algorithm finds an initial solution, which is critical for the anytime property.

**Proposed additions to Feedback:**
```
int32 tree_size_feedback           # current tree size (number of nodes)
```

This lets the client (and traces, if feedback tracepoints are enabled) see tree growth in real time.

### 2.3 Experiments Launch File

**New file:** `packages/src/experiments/launch/rrt_star.launch.py`

Mirror the `monte_carlo.launch.py` pattern: ComposableNodeContainer with server and client components, configurable executor (single/multi-threaded), server and client config file paths.

The key difference from Monte Carlo: the server config also carries RRT* parameters (`map_yaml_path`, `start_x/y`, `goal_x/y`, `step_size`, etc.).

### 2.4 Experiments Default Configs

**New files:**
- `packages/src/experiments/config/rrt_star/default_server.yaml`
- `packages/src/experiments/config/rrt_star/default_client.yaml`

### 2.5 Experiment Scripts

**New directory:** `experiments/rrt_star/`

Files to create (mirroring `experiments/monte_carlo/`):

1. **`generate_configs.py`** — Generate all configuration combinations:
   - Block sizes: `[1, 16, 64, 256, 1024, 4096, 16384]` (7 values)
   - Modes: `["reactive", "proactive"]` (2 values)
   - Threading: `["single", "multi"]` (2 values)
   - Maps: `["depot", "warehouse"]` (2 values)
   - Total: 7 × 2 × 2 × 2 = **56 configurations**
   - Each config gets a server YAML with all RRT* params + a client YAML
   - Config naming: `block_{size}_{mode}_{threading}_{map}`

2. **`run_rrt_star_experiments.sh`** — Main experiment runner:
   - Loop through all configs
   - For each config:
     - Create LTTng session
     - Enable tracepoints: framework core (`anytime_compute_entry/exit`, client/server lifecycle), overhead (`anytime_send_feedback_entry/exit`, `anytime_calculate_result_entry/exit`), and optionally `rrt_star_iteration`
     - **Important:** Unlike the Monte Carlo runner, the `send_feedback` and `calculate_result` tracepoints must NOT be commented out — they are required for overhead metrics
     - Launch via `ros2 launch experiments rrt_star.launch.py`
     - Sleep for `RUN_DURATION` seconds
     - Stop tracing, kill processes, save traces
   - Run evaluation script at end

3. **`run_quick.sh`** — Quick version:
   - 3 block sizes (1, 256, 4096), depot map only, 5s runs

4. **`test_single_config.sh`** — Sanity test with 10s on one config

5. **`evaluate_rrt_star.py`** — Evaluation and plotting script (see Section 3)

### 2.6 Docker Integration

No changes needed to Dockerfiles — the `anytime-cpu` image already has all dependencies (ROS2, LTTng, Python plotting). The `anytime_rrt_star` package builds with the same toolchain. Just ensure the workspace build includes `anytime_rrt_star`:

```bash
colcon build --packages-select anytime_rrt_star experiments
```

The `scripts/run_all.sh` top-level orchestrator should be extended with `--rrt-star` flag support, but this is optional for initial evaluation.

## 3. Metrics and Plots

### 3.1 Framework Metrics (same as Monte Carlo — from core tracepoints)

These use the **same tracepoints** as Monte Carlo (`anytime_compute_entry/exit`, `anytime_client_goal_sent/cancel_sent/goal_finished`, `anytime_server_handle_cancel`, `anytime_base_deactivate`):

| Metric | Source | Description |
|---|---|---|
| Time per block (ms) | `compute_entry` → `compute_exit` | Duration of one block of N iterations |
| Throughput (iter/sec) | block_size / (time_per_block / 1000) | Iterations per second |
| Cancellation delay (ms) | `server_handle_cancel` → `base_deactivate` | Time from cancel to deactivation |
| Goal-to-finish latency (ms) | `client_goal_sent` → `client_goal_finished` | End-to-end goal lifecycle |
| Goal-to-cancel latency (ms) | `client_goal_sent` → `client_cancel_sent` | Time client waits before cancelling |
| Cancel-to-finish latency (ms) | `client_cancel_sent` → `client_goal_finished` | Cancel round-trip time |
| Total segments completed | count of `compute_exit` events × block_size | Total iterations in run |

**Plots (grouped bar charts, block size on x-axis, 4 groups: reactive-single, reactive-multi, proactive-single, proactive-multi):**

1. `block_size_vs_time.pdf` — Time per block vs block size
2. `throughput.pdf` — Throughput vs block size
3. `cancellation_delay.pdf` — Cancellation delay vs block size
4. `goal_to_finish_latency.pdf` — Goal-to-finish latency vs block size
5. `goal_to_cancel_latency.pdf` — Goal-to-cancel latency vs block size
6. `cancel_to_finish_latency.pdf` — Cancel-to-finish latency vs block size
7. `total_iterations.pdf` — Total iterations completed vs block size
8. `total_cancellation_time.pdf` — Combined cancellation time vs block size
9. `legend.pdf` — Separate legend

**Per-map:** Generate the above plots **per map** (depot, warehouse) and also a comparison across maps.

### 3.2 Overhead and Communication Metrics

These metrics address the reviewer's concern about framework overhead — the time spent between computation blocks on communication, feedback delivery, and result processing. Each metric gets its own plot to allow independent assessment of what is worth reporting.

**Tracepoints required:** In addition to `anytime_compute_entry/exit`, these metrics require enabling:
- `anytime:anytime_send_feedback_entry` / `anytime:anytime_send_feedback_exit`
- `anytime:anytime_calculate_result_entry` / `anytime:anytime_calculate_result_exit`

These tracepoints exist in the framework but are commented out in the Monte Carlo experiment runner. The RRT* experiment runner must enable them.

| Metric | Source | Description |
|---|---|---|
| Per-block overhead (ms) | `compute_exit[i]` → `compute_entry[i+1]` | Time gap between consecutive blocks — includes feedback sending, scheduling, and any framework bookkeeping |
| Overhead ratio (%) | `overhead / (overhead + compute_time) × 100` | Fraction of wall time spent on non-computation. Key metric for the reviewer: should be negligible (<5%) for large blocks |
| Feedback send time (ms) | `send_feedback_entry` → `send_feedback_exit` | Time to serialize and publish one feedback message |
| Result compute time (ms) | `calculate_result_entry` → `calculate_result_exit` | Time to populate and send the final result after cancellation |
| Block time percentiles | Distribution of `compute_exit - compute_entry` | p50, p95, p99 of block compute time — shows timing consistency |
| Block time trend | `compute_exit - compute_entry` vs block index | Whether block compute time changes over the course of a run (e.g., due to tree growth increasing Near() cost) |

**Plots (one per metric, grouped bar charts with block size on x-axis):**

15. `per_block_overhead.pdf` — Per-block overhead (ms) vs block size. Expect overhead to be roughly constant regardless of block size, so it becomes proportionally smaller for larger blocks.

16. `overhead_ratio.pdf` — Overhead ratio (%) vs block size. This is the key plot for the reviewer. Should show overhead ratio dropping from potentially significant (small blocks) to negligible (large blocks).

17. `feedback_send_time.pdf` — Feedback send time (ms) vs block size. Should be roughly constant since feedback message size doesn't change with block size.

18. `result_compute_time.pdf` — Result compute time (ms) vs block size. May vary with tree size since result includes path extraction.

19. `block_time_percentiles.pdf` — Box plot or bar chart showing p50/p95/p99 block compute times per configuration. Reveals timing jitter and outliers.

20. `block_time_trend.pdf` — Line plot: block compute time (y-axis) vs block number within a run (x-axis). One line per block size. Shows whether RRT* tree growth causes increasing per-block cost over time (expected, since Near() is O(n log n)).

**Per-map:** Generate overhead plots per map, since tree growth characteristics differ between depot and warehouse.

### 3.3 RRT*-Specific Metrics (from rrt_star_iteration tracepoint + action result)

These require the `rrt_star_iteration` tracepoint to be enabled and/or the enhanced action result fields.

| Metric | Source | Description |
|---|---|---|
| Best path cost at cancel | Action result `result` field | Path quality achieved in one goal cycle |
| First solution iteration | Action result `first_solution_iteration` | How quickly initial path was found |
| Tree size at cancel | Action result `tree_size` | Tree complexity at end |
| Path cost convergence | `rrt_star_iteration` trace events | cost vs. iteration number |
| Cost improvement rate | Derived from convergence data | Δcost per N iterations |

**Plots:**

10. `convergence_curve.pdf` — **Key RRT* plot.** Path cost (y-axis) vs iteration (x-axis). One line per configuration (or averaged across runs). Shows the anytime property: cost decreases over time. Compare different block sizes — they should all converge to similar final costs, but the curves will have different granularity (larger blocks = less frequent feedback points).

11. `convergence_by_map.pdf` — Convergence curves comparing depot vs warehouse. Warehouse should take longer to find initial solution and converge more slowly.

12. `first_solution_iteration.pdf` — Bar chart: iteration at which first path was found, grouped by block size × mode × threading. Shows how quickly the algorithm becomes useful.

13. `best_cost_vs_block_size.pdf` — Bar chart: best path cost at cancellation vs block size. All configurations should achieve similar final costs (since the algorithm is anytime), but smaller block sizes may have slightly worse costs due to overhead.

14. `tree_size_vs_iterations.pdf` — Tree growth curve: tree size vs iterations. With pruning enabled, tree size should plateau. Without pruning, it grows linearly.

### 3.4 Collecting Convergence Data

**Challenge:** The `rrt_star_iteration` tracepoint fires once per iteration. At 100k+ iterations, this generates millions of trace events and impacts performance.

**Solutions:**

**(a) Subsampled tracepoint (recommended):** Only emit `rrt_star_iteration` every K iterations (e.g., every 100th). This is controlled by a modulo check in `compute_single_iteration()`:
```cpp
if (loop_count_ % convergence_log_interval_ == 0) {
  TRACE_RRT_STAR_ITERATION(...);
}
```
With `convergence_log_interval_` as a ROS parameter (default 100). At 100k iterations this gives 1000 trace events — very manageable.

**(b) Feedback-based collection:** The action feedback already carries `best_path_cost_`. The feedback is sent after every block. So for block_size=256, we get a cost sample every 256 iterations. This is already enough for convergence curves at block-level granularity. The evaluation script can extract cost samples from feedback tracepoints (if enabled) or from the per-block `anytime_compute_exit` events combined with the cost at that point.

**(c) Dedicated convergence run:** Run a separate short experiment with `rrt_star_iteration` enabled and small blocks (block_size=1), purely to collect detailed convergence data. This doesn't need the full parameter sweep — just one run per map.

**Recommendation:** Use approach (a) with subsampled tracepoint (every 100 iterations) for the full sweep, and approach (c) with block_size=1 for detailed convergence curves in a separate short run.

## 4. Experiment Configurations

### 4.1 Map-Specific Parameters

| Parameter | Depot | Warehouse |
|---|---|---|
| `map_yaml_path` | `maps/depot.yaml` | `maps/warehouse.yaml` |
| `start_x` | 5.0 | -10.0 |
| `start_y` | 5.0 | -20.0 |
| `goal_x` | 25.0 | 10.0 |
| `goal_y` | 10.0 | 20.0 |
| `step_size` | 0.5 | 0.5 |
| `goal_threshold` | 0.5 | 0.5 |
| `goal_bias` | 0.05 | 0.05 |
| `gamma_rrt_star` | 0.0 (auto) | 0.0 (auto) |
| `prune_interval` | 1000 | 1000 |

### 4.2 Full Sweep Matrix

| Dimension | Values | Count |
|---|---|---|
| Block size | 1, 16, 64, 256, 1024, 4096, 16384 | 7 |
| Mode | reactive, proactive | 2 |
| Threading | single, multi | 2 |
| Map | depot, warehouse | 2 |
| Runs per config | 1 (quick) / 3 (full) | - |
| Run duration | 10s (quick) / 30s (full) | - |

**Total configs:** 7 × 2 × 2 × 2 = 56
**Total runs (quick):** 56 × 1 = 56 runs × 10s ≈ 10 min + overhead
**Total runs (full):** 56 × 3 = 168 runs × 30s ≈ 85 min + overhead

### 4.3 Quick Sweep (for development/CI)

| Dimension | Values | Count |
|---|---|---|
| Block size | 1, 256, 4096 | 3 |
| Mode | reactive, proactive | 2 |
| Threading | single | 1 |
| Map | depot | 1 |
| Run duration | 5s | - |

**Total:** 3 × 2 × 1 × 1 = 6 configs × 5s ≈ 30s + overhead

### 4.4 Convergence-Only Run

Separate from the main sweep. Purpose: collect detailed cost vs. iteration data.

| Parameter | Value |
|---|---|
| Block size | 1 |
| Mode | reactive |
| Threading | single |
| Maps | depot, warehouse |
| Duration | 60s |
| `convergence_log_interval` | 1 (every iteration) |
| Tracepoints | `anytime:rrt_star_iteration` enabled |

**Total:** 2 runs × 60s = 2 min

## 5. Evaluation Script Design (`evaluate_rrt_star.py`)

### 5.1 Trace Parsing

Same babeltrace parsing as `evaluate_monte_carlo.py`. Events of interest:

**Framework events** (always enabled):
- `anytime:anytime_compute_entry` / `anytime:anytime_compute_exit`
- `anytime:anytime_client_goal_sent` / `anytime:anytime_client_cancel_sent` / `anytime:anytime_client_goal_finished`
- `anytime:anytime_server_handle_cancel` / `anytime:anytime_base_deactivate`
- `anytime:anytime_send_feedback_entry` / `anytime:anytime_send_feedback_exit` (for overhead metrics)
- `anytime:anytime_calculate_result_entry` / `anytime:anytime_calculate_result_exit` (for overhead metrics)

**RRT*-specific events** (optionally enabled):
- `anytime:rrt_star_iteration` — carries `iteration_num`, `tree_size`, `best_cost`
- `anytime:rrt_star_result` — carries `best_cost`, `total_iterations`, `tree_size`

### 5.2 Metrics Extraction

Same structure as `extract_metrics_from_events()` in Monte Carlo evaluation, with additional fields:

```python
metrics = {
    # ... same framework metrics as Monte Carlo ...

    # Overhead metrics (from compute_entry/exit + send_feedback + calculate_result)
    'per_block_overheads': [],        # ms gaps: compute_exit[i] → compute_entry[i+1]
    'overhead_ratios': [],            # per-block: overhead / (overhead + compute_time) × 100
    'feedback_send_times': [],        # ms: send_feedback_entry → send_feedback_exit
    'result_compute_times': [],       # ms: calculate_result_entry → calculate_result_exit
    'block_times': [],                # ms: all individual block compute times (for percentiles)

    # RRT*-specific (from rrt_star_iteration events)
    'convergence_data': [],           # list of (iteration, best_cost, tree_size)
    # RRT*-specific (from rrt_star_result events)
    'final_best_costs': [],           # best cost at each goal cycle
    'final_tree_sizes': [],           # tree size at each goal cycle
}
```

### 5.3 Output Files

```
experiments/rrt_star/
├── configs/                          # generated YAML files
├── traces/                           # raw LTTng trace directories
└── results/
    ├── individual_runs.csv           # per-run metrics
    ├── aggregated_results.csv        # averaged across runs
    ├── aggregated_results.json       # JSON format
    ├── convergence_data/             # per-config convergence CSVs
    │   ├── depot_convergence.csv
    │   └── warehouse_convergence.csv
    └── plots/                        # all PDF plots
        ├── block_size_vs_time.pdf
        ├── throughput.pdf
        ├── cancellation_delay.pdf
        ├── goal_to_finish_latency.pdf
        ├── goal_to_cancel_latency.pdf
        ├── cancel_to_finish_latency.pdf
        ├── total_iterations.pdf
        ├── total_cancellation_time.pdf
        ├── convergence_curve.pdf
        ├── convergence_by_map.pdf
        ├── first_solution_iteration.pdf
        ├── best_cost_vs_block_size.pdf
        ├── tree_size_vs_iterations.pdf
        ├── per_block_overhead.pdf
        ├── overhead_ratio.pdf
        ├── feedback_send_time.pdf
        ├── result_compute_time.pdf
        ├── block_time_percentiles.pdf
        ├── block_time_trend.pdf
        └── legend.pdf
```

## 6. Implementation Sequence

### Phase 1: Tracing Integration
1. Add `rrt_star_*` tracepoints to `anytime_tracing` (tp_call.h, anytime_tracetools.h, anytime_tracetools.c)
2. Update `anytime_rrt_star/tracing.hpp` with real macros
3. Add `convergence_log_interval` parameter to management class (subsampled tracepoint)

### Phase 2: Action Interface Enhancement
4. Add `tree_size` and `first_solution_iteration` to `RrtStar.action` Result
5. Add `tree_size_feedback` to `RrtStar.action` Feedback
6. Update `populate_result()` and `populate_feedback()` in management class

### Phase 3: Experiment Infrastructure
7. Create `experiments/launch/rrt_star.launch.py`
8. Create `experiments/config/rrt_star/default_server.yaml` and `default_client.yaml`
9. Create `experiments/rrt_star/generate_configs.py`
10. Create `experiments/rrt_star/run_rrt_star_experiments.sh`
11. Create `experiments/rrt_star/run_quick.sh`
12. Create `experiments/rrt_star/test_single_config.sh`

### Phase 4: Evaluation
13. Create `experiments/rrt_star/evaluate_rrt_star.py`
14. Verify full pipeline: generate → run → trace → evaluate → plot

### Phase 5: Validation
15. Run quick sweep, verify all plots generated correctly
16. Run convergence-only experiment, verify convergence curves
17. Compare depot vs warehouse results
