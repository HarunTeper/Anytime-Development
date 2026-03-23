# YOLO Experiment -- Methodology

## 1. Experiment Purpose

This experiment evaluates the **cancellation performance** of an anytime YOLO object
detection system. The model has 22 neural network layers that can be interrupted at
layer boundaries; earlier layers yield faster but lower-quality detections, later layers
yield better results at higher cost.

Two questions are addressed:

1. **System characterization** (Phase 1): How does detection quality evolve across
   layers, what is the per-layer computation cost, and how does throughput vary across
   execution models (sync/async, single/multi-threaded)?
2. **Cancellation effectiveness** (Phase 2): Given score-based early cancellation,
   how does block size affect cancellation delay, total runtime, and layers processed?

## 2. System Architecture

Three ROS 2 components form the processing pipeline:

1. **Video Publisher** (`video_publisher.py`) -- Loads numbered JPG images from a
   directory, publishes them one at a time to the `video_frames` topic. Publishing
   is trigger-based: the next image is sent only after the client publishes the
   detection result of the current image to the `images` topic. This enforces
   sequential, one-image-at-a-time processing.

2. **Anytime Client** (`anytime_client.cpp`) -- Subscribes to `video_frames`, sends
   each image as a ROS 2 action goal to the server. Receives intermediate feedback
   after each block of layers. Inspects feedback for cancellation criteria (score-based
   or hard deadline) and sends cancel requests when satisfied. After receiving the
   result, publishes to `images` to trigger the next image.

3. **Anytime Server** (`anytime_management.hpp`, `anytime_base.hpp`) -- Runs the
   22-layer YOLO model. Computes layers in configurable block sizes. Emits feedback
   containing intermediate detection results after each block. On cancellation, calls
   `calculate_result()` and returns the latest intermediate result.

## 3. Independent Variables

### Phase 1 (System Characterization)

| Variable | Values | Rationale |
|----------|--------|-----------|
| Sync mode | sync, async | Sync blocks on each GPU layer; async pipelines submissions via CUDA callbacks |
| Threading | single, multi | Single-threaded uses one executor thread; multi-threaded allows parallel callback processing |

Phase 1 baseline uses fixed block_size=1. Phase 3 throughput uses fixed block_size=22.

### Phase 2 (Cancellation Experiments)

| Variable | Values | Rationale |
|----------|--------|-----------|
| Block size | 1, 4, 8, 11, 16, 22 | 1=max granularity (22 loops), 4=mid-range (6 loops, hits L8 quality jump), 8=3-loop transition, 11=first 2-loop config ("knee" of runtime curve), 16=2-loop with stable 98% quality (low variance), 22=full block baseline |
| Sync mode | sync, async | Same as Phase 1 |
| Threading | single, multi | Same as Phase 1 |

Total Phase 2 configurations: 6 block sizes x 2 sync modes x 2 threading modes = 24.

## 4. Controlled Variables

| Variable | Value | Notes |
|----------|-------|-------|
| Anytime mode | proactive | Reactive mode does not support score-based cancellation feedback (see Design Decisions) |
| Total layers | 22 | Fixed YOLO architecture (22 layer chunks, indices 0-21) |
| Image set | ~190 images from `packages/src/video_publisher/images/` | Sorted by filename, same order every trial |
| Image resolution | 640x640 (resized internally) | Input images normalized and resized before GPU inference |
| Target class | 9 (traffic light, COCO) | Used for score-based cancellation and quality filtering |
| Score threshold | 0.8 | Detection confidence required to trigger early cancellation |
| cancel_after_layers | 22 | Hard deadline equals total layers, so score-only cancellation in Phase 2 |
| Warmup images | 5 per trial | Skipped in analysis to exclude GPU JIT compilation overhead |
| Trials | 5 per configuration | For statistical variance estimation; consistent with RRT* experiment |
| Weights | `packages/src/anytime_yolo/weights_32/` | TensorRT FP32 weights, engines cached after first build |
| Init wait | 3 seconds | Delay between YOLO launch and video publisher start |
| Between-trial pause | 5 seconds | |
| Between-config pause | 10 seconds | |

## 5. Warmup Filtering

### Problem
The first images processed after GPU initialization incur TensorRT JIT compilation
overhead, producing timing measurements 2-10x longer than steady-state. For ~190
images per trial, 5 noisy samples represent ~2.5% of data but can dominate
max/mean statistics.

### Approach: Fixed Count Cutoff
The first 5 images (goals) per trial are discarded in all analysis scripts.

**Rationale:**
- TensorRT engine optimization completes within the first few inferences.
- A fixed count is simple and reproducible. The count of 5 was chosen empirically
  to be safely past JIT completion on the target hardware.
- Applied consistently across all analysis scripts (`2a_analyze_quality.py`,
  `2b_analyze_blocks.py`, `4_analyze_throughput.py`, `7_analyze_cancellation.py`).

### Effect on Metrics
After filtering, per-goal tracking starts fresh. No artificial period or delta is
introduced at the filtering boundary.

## 6. Phase 1: System Characterization

### Step 0: Setup and Config Generation (`0_test_setup.sh`)

Quick environmental verification:
- Launches YOLO components with default config (block_size=1, proactive)
- Runs for 30 seconds maximum with LTTng tracing
- Verifies trace events are captured
- Also generates config files for Steps 1 and 3

### Step 1: Baseline Collection (`1_collect_baseline.sh`)

**Purpose:** Collect per-layer timing and detection data with all 22 layers.

| Parameter | Value |
|-----------|-------|
| block_size | 1 (layer-by-layer) |
| Mode | proactive |
| Sync | sync |
| Threading | single |
| Cancellation | none |
| Trials | 5 |

**Config files:** `configs/phase1_server.yaml`, `configs/phase1_client.yaml`

**Output:** `traces/phase1_baseline_trial{1,2,3,4,5}/`

**LTTng setup:**
- Session name: `yolo_phase1_baseline`
- Events enabled: `anytime:*` (all anytime provider tracepoints)
- One session per trial; old trace directories removed before each trial

**Tracepoints used downstream:**
- `anytime:anytime_base_activate` -- goal start, separates images
- `anytime:yolo_layer_start(layer_num)` -- layer N submitted (0-indexed)
- `anytime:yolo_layer_end(layer_num)` -- layer N completed (1-indexed after increment)
- `anytime:yolo_exit_calculation_start(layer_num)` -- exit calc begins
- `anytime:yolo_exit_calculation_end(layer_num, num_detections)` -- exit calc ends
- `anytime:yolo_detection(layer_num, class_id, confidence, ...)` -- individual detection
- `anytime:yolo_result(processed_layers, total_detections)` -- final result

### Step 2a: Quality Analysis (`2a_analyze_quality.py`)

**Purpose:** Determine how detection quality evolves across layers, identify optimal
cancellation points.

**Input:** Phase 1 baseline traces.

**Configurable parameters:**
```
FILTER_BY_CLASS = True      # Filter detections to target class only
TARGET_CLASS_ID = 9         # Traffic light (COCO)
WARMUP_IMAGES = 5           # Skip first 5 images per trial
```

**Per-image data extraction:**
- Layer detections: count of `yolo_detection` events per layer (filtered by class if enabled)
- Layer computation time: `yolo_layer_end.timestamp - yolo_layer_start[layer_num-1].timestamp`
  (nanoseconds, converted to ms)
- Exit calculation time: `yolo_exit_calculation_end.timestamp - yolo_exit_calculation_start.timestamp`
  (nanoseconds, converted to ms)
- Final detections: from `yolo_result` event's `total_detections` field (or filtered count
  at max layer if class filtering is enabled)

**Metrics computed:**

| Metric | Formula | Scope |
|--------|---------|-------|
| Detection progression | `mean(detection_count_at_layer_L)` across images | Images with detections only |
| Quality ratio | `detections_at_layer_L / final_detections` | Images with detections only |
| Threshold layer | First layer L where `quality_ratio >= threshold` | Per-image, then aggregated |
| Layer computation time | `layer_end_ts - layer_start_ts` | All images |
| Exit calculation time | `exit_calc_end_ts - exit_calc_start_ts` | All images |

Quality thresholds evaluated: 50%, 60%, 70%, 80%, 90%, 95%, 99%.

**Filtering logic:**
- Quality ratio and threshold calculations: only `images_with_detections`
  (images where `final_detections > 0`).
- Timing calculations: all images (timing is meaningful regardless of detection count).
- Detection counting: if `FILTER_BY_CLASS=True`, only `yolo_detection` events with
  `class_id == TARGET_CLASS_ID` are counted.

**Plots generated:**

| Plot | X-axis | Y-axis | Type |
|------|--------|--------|------|
| `detection_progression.pdf` | Layer | Avg detection count | Line + error bars (std) |
| `quality_ratio_progression.pdf` | Layer | Quality ratio | Bar + error bars; 90/95/99% reference lines |
| `cancellation_histograms.pdf` | Layer | Image count | 5 histograms (70/80/90/95/99%) with mean+median lines |
| `quality_boxplot.pdf` | Layer | Quality ratio distribution | Box plot per layer |
| `layer_computation_times.pdf` | Layer | Time (ms) | Line + error bars |
| `exit_calculation_times.pdf` | Layer | Time (ms) | Line + error bars |
| `combined_timing.pdf` | Layer | Time (ms) | Side-by-side + stacked bars (computation vs exit) |
| `cumulative_timing.pdf` | Layer | Cumulative time (ms) | Line + stacked area |

**Output:** `results/quality_analysis/` (plots + `quality_analysis.json` + `quality_summary.txt`)

### Step 2b: Block Size Analysis (`2b_analyze_blocks.py`)

**Purpose:** Evaluate the trade-off between block size and cancellation responsiveness.

**Input:** Same Phase 1 baseline traces as Step 2a.

**Parameters:**
```
WARMUP_IMAGES = 5
MAX_LAYER = 22
```

**Block delay calculation:**

For block size N, layers are grouped sequentially: `[0..N-1], [N..2N-1], ...`

Per block:
```
block_computation = sum(layer_computation_time[l] for l in block_layers)
block_exit        = exit_calculation_time[last_layer_in_block]
block_delay       = block_computation + block_exit
```

Per image:
```
total_delay = sum(block_delay for each block)
max_block_delay = max(block_delay for each block)
```

**Metrics per block size:**

| Metric | Definition |
|--------|------------|
| Total delay | Sum of all block delays for processing all 22 layers |
| Max per-block delay | Worst-case single block delay (cancellation responsiveness bound) |
| Number of blocks | `ceil(22 / block_size)` |

Statistics computed: mean, std, min, max, median across all images (all trials pooled).

**Plots generated:**

| Plot | Description |
|------|-------------|
| `block_size_delays.png` | Total delay vs block size (0-22), with block_size=1 reference line |
| `max_block_delay.png` | Max per-block delay vs block size |
| `num_blocks_vs_block_size.png` | Number of blocks vs block size |
| `per_block_delay_distribution.png` | Histograms for block sizes [1, 3, 5, 8, 16, 22] |
| `detailed_block_breakdown.png` | Per-block delay bars + cumulative line for sizes [1, 4, 8, 16] |

**Output:** `results/block_analysis/` (plots + `block_analysis.json` + `block_summary.txt`)

### Step 3: Throughput Measurement (`3_measure_throughput.sh`)

**Purpose:** Measure maximum processing throughput under 4 execution configurations
(all layers, no cancellation).

| Parameter | Value |
|-----------|-------|
| block_size | 22 (all layers in one block) |
| Mode | proactive |
| Cancellation | none |
| Trials | 3 per config |

**Configurations tested:**

| Config | Sync | Threading |
|--------|------|-----------|
| 1 | sync | single |
| 2 | sync | multi |
| 3 | async | single |
| 4 | async | multi |

**Config files:** `configs/phase3_server_{sync|async}_{single|multi}.yaml`,
`configs/phase3_client.yaml`

**Output:** `traces/phase3_{sync|async}_{single|multi}_trial{1,2,3,4,5}/`

**LTTng setup:** Same pattern as Step 1. Session name: `yolo_phase3`.

### Step 4: Throughput Analysis (`4_analyze_throughput.py`)

**Purpose:** Compare throughput and per-layer timing across the 4 execution configurations.

**Input:** Phase 3 traces.

**Parameters:**
```
WARMUP_IMAGES = 5
```

**Per-goal metrics extracted:**
- Goal time: `(goal_end - goal_start) / 1e6` ms
- Per-layer computation time (sync mode only): `(yolo_layer_end.ts - yolo_layer_start[prev].ts) / 1e6` ms
- Per-layer exit calculation time (sync mode only): `(exit_calc_end.ts - exit_calc_start.ts) / 1e6` ms
- Final exit cost: `(goal_end - last_layer_end) / 1e6` ms

Note: per-layer timing is unavailable in async mode because GPU pipelining
makes individual layer boundaries indeterminate at the CPU level. Plots that
show per-layer breakdowns (e.g., `layer_computation_by_config.png`) include
only sync configurations.

**Aggregate metrics per configuration:**

| Metric | Formula |
|--------|---------|
| Avg goal time | `mean(goal_times)` ms |
| Throughput | `num_goals / (sum(goal_times) / 1000)` images/sec |
| Per-layer computation | `mean(comp_time[layer])` ms (sync only) |
| Per-layer exit cost | `mean(exit_time[layer])` ms (sync only) |

**Plots generated:**

| Plot | Description |
|------|-------------|
| `total_goal_time_comparison.png` | Bar chart: avg goal time per config |
| `throughput_comparison.png` | Bar chart: images/sec per config |
| `layer_computation_by_config.png` | Line plots: per-layer computation time per config |
| `exit_calculation_by_config.png` | Per-layer exit time + final exit cost comparison |
| `stacked_layer_times_by_config.png` | Stacked bars: computation + exit per layer per config |
| `combined_stacked_comparison.png` | Side-by-side stacked bars across configs |
| `cumulative_runtime.png` | Cumulative time progression per config |

**Output:** `results/runtime_analysis/` (plots + `runtime_analysis.json` + `runtime_summary.txt`)

**Manual review point:** Outputs from Steps 2a, 2b, and 4 should be reviewed before
proceeding to Phase 2. These inform the choice of block sizes, score threshold,
and expected cancellation behavior.

## 7. Phase 2: Cancellation Experiments

### Step 5: Configuration Generation (`5_generate_configs.py`)

Generates YAML config files for all Phase 2 combinations.

**Server config parameters (per file):**
```yaml
anytime_server:
  ros__parameters:
    is_reactive_proactive: "proactive"
    multi_threading: true/false
    block_size: 1/4/8/13/16/20/22
    is_sync_async: "sync"/"async"
    weights_path: <absolute path to weights_32>
```

**Client config (shared, 1 file):**
```yaml
anytime_client:
  ros__parameters:
    image_topic: "video_frames"
    cancel_after_layers: 22
    cancel_layer_score: true
    score_threshold: 0.8
    target_class_id: "9"
```

**Output:** `configs/phase4_server_bs{N}_{mode}_{sync}_{threading}.yaml` (24 files)
and `configs/phase4_client.yaml` (1 file).

### Step 6: Experiment Execution (`6_run_experiments.sh`)

**Pre-experiment setup:**
1. Source ROS 2 workspace
2. Run prerequisite check (`check_yolo_prerequisites.sh`)
3. Warm up TensorRT engines (`warmup_yolo_engines.sh`)
4. Write results summary header with client config values

**Experiment loop:** For each (block_size, mode, sync_mode, threading) x 5 trials:

1. Create LTTng session (`yolo_phase4`) with per-trial output directory
2. Enable events: `lttng enable-event -u 'anytime:*'`
3. Start tracing
4. Launch YOLO server/client with the trial's server config and shared client config
5. Wait 3 seconds for initialization
6. Launch video publisher
7. Wait for video publisher to exit (all images processed)
8. Stop tracing, wait 1 second for buffer flush
9. Kill all processes (SIGTERM then SIGKILL)
10. Destroy LTTng session
11. Log trial completion to results file

**Output:** `traces/phase4_bs{N}_proactive_{sync}_{threading}_trial{1,2,3,4,5}/`

**Process management:** Explicit kill of `component_container`, `anytime_yolo`,
`video_publisher`, and `ros2` processes. Two-stage termination: SIGTERM first,
SIGKILL after 1 second.

### Step 7: Cancellation Analysis (`7_analyze_cancellation.py`)

**Purpose:** Evaluate cancellation performance across all Phase 2 configurations.

**Input:** Phase 4 traces + `configs/phase4_client.yaml` (for `cancel_after_layers`
and `score_threshold` values).

**Parameters:**
```
WARMUP_IMAGES = 5
```

**Configuration parsing:** Extracted from trace directory name via regex:
`phase4_bs{size}_{mode}_{sync}_{threading}_trial{N}`

**Per-goal data extraction:**

| Event | Data extracted |
|-------|----------------|
| `anytime:anytime_base_activate` | Goal start timestamp; separates goals |
| `anytime:anytime_client_cancel_sent` | Cancel request timestamp |
| `anytime:anytime_client_goal_finished` | Result received timestamp |
| `anytime:yolo_layer_start(layer_num)` | Layer start timestamp |
| `anytime:yolo_layer_end(layer_num)` | Layer end timestamp; `layers_processed = max(layer_num)` |
| `anytime:yolo_exit_calculation_end(layer_num, num_detections)` | Exit calc timestamp; detection count per layer |

**Per-goal metrics:**

| Metric | Formula | Conditions |
|--------|---------|------------|
| Cancellation delay | `(result_received - cancel_sent) / 1e6` ms | Only when cancel was sent |
| Total runtime | `(result_received - goal_start) / 1e6` ms | All goals |
| Layers processed | `max(layer_num from yolo_layer_end events)` | All goals |
| Detections at cancel | `layer_detections[max_layer]` from `yolo_exit_calculation_end` | When data available |
| Score-cancelled flag | `cancel_sent != None AND layers_processed < cancel_after_layers` | Requires cancel_after_layers |

**Population separation:**

Goals are classified into two populations:
- **Score-cancelled**: Client detected a qualifying traffic light (score >= 0.8)
  and sent cancel before layer 22. `layers_processed < cancel_after_layers`.
- **Hard-deadline**: No qualifying detection found. All 22 layers completed.
  `layers_processed >= cancel_after_layers` or no cancel sent.

This separation prevents blending early-exit images (where cancellation is meaningful)
with full-run images (where cancellation delay is simply total runtime).

**Aggregation:** Metrics grouped by configuration key
(`bs{N}_{mode}_{sync}_{threading}`), pooling all goals across 5 trials.

**Statistics per configuration:**

| Statistic | Applied to |
|-----------|------------|
| mean, std, min, max, median | Cancellation delay, total runtime, layers processed |
| mean, std | Detections at cancellation |
| count | Score-cancelled goals, hard-deadline goals |
| mean | Score-cancelled delay, runtime, layers; hard-deadline runtime, layers |

**Plots generated:**

| Plot | Description |
|------|-------------|
| `cancellation_delay_comparison.pdf` | Grouped bars: cancellation delay by block size, 4 bars per block size (sync-single, sync-multi, async-single, async-multi) |
| `total_runtime_comparison.pdf` | Same layout: total runtime |
| `layers_processed_comparison.pdf` | Same layout: layers processed, with cancel_after_layers reference line |
| `metrics_by_block_size.pdf` | 3 subplots: delay, runtime, layers, grouped by block size |
| `distribution_boxplots.pdf` | 3 stacked boxplots: distributions across all configurations |
| `legend.pdf` | Standalone legend for LaTeX compositing |

**Plot styling:**
```
PLOT_WIDTH = 12          PLOT_HEIGHT = 8
PLOT_DPI = 300           FONT_SIZE_LABEL = 30
FONT_SIZE_TICK_LABELS = 30   LEGEND_SIZE = 30
MARKER_SIZE = 12         LINE_WIDTH = 2
CAPSIZE = 5
```

Colors for configuration combinations: blue (#1f77b4), orange (#ff7f0e),
green (#2ca02c), red (#d62728). Colors for block sizes: dynamically generated
from `plt.cm.tab10` colormap.

**Output files:**

| File | Content |
|------|---------|
| `results/phase4_analysis/phase4_analysis.json` | Per-configuration metrics, population breakdown, detection stats |
| `results/phase4_analysis/phase4_summary.txt` | Human-readable summary sorted by runtime, with best-configuration recommendations |
| `results/phase4_analysis/*.pdf` | All plots listed above |

## 8. Cancellation Mechanism (Implementation Detail)

### Server-Side: Proactive Mode

The proactive execution loop (`anytime_base.hpp`):
1. Drain GPU completions (async mode)
2. Compute block (N layers, where N = block size)
3. Drain GPU completions again
4. Check `goal_handle_->is_canceling()` and `should_finish()`
5. If cancelling/finished: `calculate_result()`, return result, deactivate
6. Else: `calculate_result()`, `send_feedback()`, loop

Key: In proactive mode, `calculate_result()` runs BEFORE `send_feedback()`, so
feedback always contains the freshest intermediate result. This is essential for
score-based cancellation to work correctly.

### Client-Side: Feedback Processing

On each feedback reception (`anytime_client.cpp:process_feedback()`):

1. **Score-based check** (if `cancel_layer_score=true`):
   - Iterate through `feedback->detections[]`
   - For each detection, check all `results[]` for `score >= score_threshold`
     AND `class_id == target_class_id`
   - If found and `!is_cancelling_`: trigger cancel

2. **Hard deadline check** (always active):
   - If `feedback->processed_layers >= cancel_after_layers` and `!is_cancelling_`:
     trigger cancel

3. **Cancel execution:**
   - Set `is_cancelling_ = true` (guard against duplicate cancel)
   - Emit `TRACE_ANYTIME_CLIENT_CANCEL_SENT(timestamp)`
   - Call `action_client_->async_cancel_goal()`

### Sync vs Async Layer Computation

**Sync mode** (`anytime_management.hpp`):
- `inferStep()` is a blocking GPU call
- `processed_layers` incremented immediately after each step
- `send_feedback()` called after each block of layers
- Tracing: `yolo_layer_start(N)` before step, `yolo_layer_end(N+1)` after

**Async mode:**
- `inferStep()` registers a CUDA callback for GPU completion
- Callback increments atomic `completion_signals_`
- `process_gpu_completions()` drains signals on executor thread,
  incrementing `processed_layers` and emitting `yolo_layer_end` per completion
- Feedback sent after draining all completions in the current block

### Why Proactive Mode Only

Reactive mode (`anytime_base.hpp:54-100`) sends feedback AFTER computing a block
but does NOT call `calculate_result()` before feedback. The client receives stale
intermediate results, making score-based cancellation unreliable. Proactive mode
ensures fresh results in every feedback message.

## 9. Tracing Methodology

- **Tool:** LTTng userspace tracing (UST). Near-zero overhead; no probe effect
  on timing measurements.
- **Events enabled:** `anytime:*` (all anytime provider tracepoints)
- **Session management:** One LTTng session per trial. Old trace directories
  removed before each trial to prevent stale data.
- **Buffer flush:** Tracing stopped before killing processes; 1-second delay
  between stop and kill allows buffer flush to complete.

### Tracepoints Used by Analysis Scripts

| Tracepoint | Fields | Used in |
|------------|--------|---------|
| `anytime:anytime_base_activate` | (none) | All scripts: goal boundary |
| `anytime:yolo_layer_start` | layer_num | Steps 2a, 2b, 4: layer timing |
| `anytime:yolo_layer_end` | layer_num | Steps 2a, 2b, 4, 7: layer timing, layer count |
| `anytime:yolo_exit_calculation_start` | layer_num | Steps 2a, 2b, 4: exit timing |
| `anytime:yolo_exit_calculation_end` | layer_num, num_detections | Steps 2a, 2b, 4, 7: exit timing, detection count |
| `anytime:yolo_detection` | layer_num, class_id, confidence, bbox | Step 2a: per-detection counting |
| `anytime:yolo_result` | processed_layers, total_detections | Steps 2a, 4: final result |
| `anytime:anytime_compute_exit` | iterations_completed, computation_time_ns | Step 4: block completion |
| `anytime:anytime_client_cancel_sent` | timestamp_ns | Step 7: cancel timing |
| `anytime:anytime_client_goal_finished` | timestamp_ns, result_code | Step 7: result timing |

### Layer Numbering Convention

In both sync and async modes, the C++ instrumentation emits:
- `yolo_layer_start(N)` — before GPU step, where N = 0..24 (pre-increment)
- `yolo_layer_end(N+1)` — after GPU step, where N+1 = 1..22 (post-increment)

All analysis scripts account for this offset: when processing a `yolo_layer_end`
event with `layer_num=K`, the matching start time is looked up at `layer_num=K-1`.

### Trace Parsing

Shared library `trace_utils.py` provides:
- `parse_trace_directory(path)`: runs `babeltrace`,
  filters for `anytime:` events, extracts timestamp (nanosecond precision from
  `[HH:MM:SS.nanoseconds]` format), event name, and field key-value pairs.
- Returns list of `TraceEvent` objects (timestamp, event_name, fields dict).
- Reports a warning if any trace lines could not be parsed, with a count of
  skipped lines.

The `verify_tracepoints.sh` script also uses `babeltrace` consistently with the
analysis scripts.

## 10. Statistical Methodology

### Aggregation Across Trials
- Goals from all 5 trials per configuration are **pooled** into a single list.
- Summary statistics (mean, std, min, max, median) computed on the pooled data.
- This provides larger sample sizes (~555 goals per config after warmup removal)
  at the cost of masking trial-to-trial variability.

### Population-Separated Reporting
For Phase 2, each configuration reports two sets of metrics:
- **Score-cancelled goals**: only goals where `cancel_sent AND layers_processed < 22`
- **Hard-deadline goals**: all remaining goals
- Combined metrics are also reported for backward compatibility.

### Quality Ratio Statistics
In Step 2a, quality ratio is only computed for images with `final_detections > 0`.
Images without target-class detections are excluded from quality analysis but
included in timing statistics.

## 11. Output Files

### Phase 1 Outputs

| Directory | Contents |
|-----------|----------|
| `results/quality_analysis/` | 8 PDF plots, `quality_analysis.json`, `quality_summary.txt` |
| `results/block_analysis/` | 5 PNG plots, `block_analysis.json`, `block_summary.txt` |
| `results/runtime_analysis/` | 7 PNG plots, `runtime_analysis.json`, `runtime_summary.txt` |

### Phase 2 Outputs

| Directory | Contents |
|-----------|----------|
| `results/phase4_analysis/` | 6 PDF plots, `phase4_analysis.json`, `phase4_summary.txt` |
| `results/phase4_experiment_summary.txt` | Per-trial completion log from experiment runner |

### Trace Data

| Directory pattern | Content |
|-------------------|---------|
| `traces/phase1_baseline_trial{1,2,3,4,5}/` | Baseline layer-by-layer traces |
| `traces/phase3_{sync\|async}_{single\|multi}_trial{1,2,3,4,5}/` | Throughput traces |
| `traces/phase4_bs{N}_proactive_{sync}_{threading}_trial{1,2,3,4,5}/` | Cancellation traces |

## 12. Design Decisions Log

| # | Decision | Rationale |
|---|----------|-----------|
| 1 | Proactive mode only for Phase 2 | Reactive mode does not call `calculate_result()` before `send_feedback()`, making score-based cancellation unreliable |
| 2 | Block sizes [1, 4, 8, 11, 16, 22] | Selected based on full 1-22 sweep: 1=22 loops (overhead reference), 4=6 loops (hits L8 jump), 8=3 loops (transition), 11=2 loops (runtime knee), 16=2 loops (stable 98% quality, low variance vs L15), 22=1 loop (baseline) |
| 3 | cancel_after_layers = 22 (score-only) | Isolates score-based cancellation behavior without hard-deadline interference |
| 4 | Score threshold 0.8, class 9 | Simulates targeted detection (traffic light) with high confidence requirement |
| 5 | 5 warmup images, fixed count | GPU JIT completes within first few inferences; fixed count is simple and reproducible |
| 6 | 5 trials per config | Consistent with RRT* experiment; provides tighter confidence intervals (~935 goals per config) |
| 7 | Sequential image processing | Isolates per-image cancellation from pipeline effects; ensures measurement independence |
| 8 | Population separation (score-cancelled vs hard-deadline) | Prevents blending early-exit responsiveness metrics with full-run images |
| 9 | Detection count at cancellation from `yolo_exit_calculation_end` | Captures quality at the actual cancellation point for quality-latency trade-off analysis |
| 10 | Dynamic block size colors via `plt.cm.tab10` | Handles variable number of block sizes without hardcoded color mappings |
| 11 | Trigger-based image publishing | Ensures sequential processing; next image only after current result received |
| 12 | Two-stage process kill (SIGTERM then SIGKILL) | Graceful shutdown attempt before forced kill; prevents orphan processes |
| 13 | LTTng stop before process kill | Ensures trace buffers are flushed before processes terminate |
| 14 | Separate legend PDF | Enables LaTeX figure compositing with independent legend placement |
