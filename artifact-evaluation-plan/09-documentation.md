# 09 — Documentation

**Day:** 4 (Fri Feb 27)
**Priority:** HIGH
**Estimated time:** 4-5 hours
**Depends on:** 01 (GPU sync fix — for documenting the new mechanism)

---

## Overview

Create three documentation files covering architecture, component details, and experiment configuration. These are critical for artifact evaluation — reviewers need to understand the system without reading all the source code.

---

## 9.1 Architecture Documentation

### New File: `docs/ARCHITECTURE.md`

#### Contents

**1. System Overview**

ASCII diagram showing the high-level architecture:

```text
┌─────────────────────────────────────────────────────┐
│                    ROS 2 Executor                    │
│                                                      │
│  ┌──────────────┐    Action    ┌──────────────────┐ │
│  │ Anytime      │◄────────────►│ Anytime          │ │
│  │ Client       │  Goal/Cancel │ Server           │ │
│  │              │  Feedback    │                   │ │
│  └──────────────┘  Result      │  ┌────────────┐  │ │
│                                │  │ Anytime    │  │ │
│                                │  │ Waitable   │  │ │
│                                │  │ (Guard     │  │ │
│                                │  │ Condition) │  │ │
│                                │  └─────┬──────┘  │ │
│                                │        │         │ │
│                                │  ┌─────▼──────┐  │ │
│                                │  │ Anytime    │  │ │
│                                │  │ Base       │  │ │
│                                │  │ (Compute)  │  │ │
│                                │  └─────┬──────┘  │ │
│                                └────────┼─────────┘ │
└─────────────────────────────────────────┼───────────┘
                                          │
                              ┌───────────▼───────────┐
                              │  Domain Implementation │
                              │  (Monte Carlo / YOLO)  │
                              └────────────────────────┘
```

**2. Package Dependency Graph**

```text
anytime_interfaces (action definitions)
       │
anytime_tracing (LTTng tracepoints)
       │
anytime_core (AnytimeBase, AnytimeWaitable, AnytimeServer, AnytimeClientBase)
       │              \
anytime_monte_carlo   anytime_yolo (+ TensorRT GPU)
       │                    │
interference          video_publisher
       \                   /
        experiments (launch files + configs)
```

**3. Key Design Patterns**

- **Template Method Pattern:** `AnytimeBase` defines the algorithm skeleton, derived classes implement `compute_single_iteration()`, `should_finish()`, `populate_result()`, `populate_feedback()`
- **Template-based Mode Selection:** Compile-time `isReactiveProactive` and `isSyncAsync` template parameters enable zero-overhead mode switching
- **Guard Condition Pattern:** `AnytimeWaitable` uses ROS 2 guard conditions to wake the executor when anytime computation needs scheduling — avoids polling
- **Cooperative Notification:** GPU async mode uses atomic counters + CUDA events for thread-safe completion signaling (see Section VI-B in paper)

**4. Execution Flow — Reactive Mode**

```text
Goal received → activate()
  → compute_single_iteration() × batch_size
  → send_feedback()
  → if should_finish(): calculate_result() → succeed()
  → else: re-trigger waitable for next batch
  → if cancel_requested: calculate_result() → cancel()
```

**5. Execution Flow — Proactive Mode**

```text
Goal received → activate()
  → compute_single_iteration() × batch_size
  → calculate_result()     ← pre-computed every batch
  → send_feedback()
  → if should_finish(): succeed()
  → else: re-trigger waitable for next batch
  → if cancel_requested: return pre-computed result → cancel()
```

**6. GPU Async Synchronization** (new, from step 01)

```text
GPU Thread                    Executor Thread
─────────                     ───────────────
Layer completes
├─ atomic increment signal
├─ trigger guard condition ──► is_ready() = true
│                              execute() called
│                              ├─ atomic read+reset signals
│                              ├─ check CUDA events in order
│                              ├─ process completed layers
│                              └─ send_feedback() per layer
```

**7. Tracing Architecture**

- 40+ LTTng tracepoints covering compute entry/exit, feedback, result, cancellation, layer start/end
- Tracepoints mapped to paper figures (Table mapping which tracepoints produce which measurements)

---

## 9.2 Experiment Documentation

### New File: `docs/EXPERIMENTS.md`

#### Contents

**1. Experiment Matrix**

| Experiment | Configs | Trials | Duration | Hardware |
| ---- | ---- | ---- | ---- | ---- |
| Monte Carlo | 28 (7 batch × 2 mode × 2 thread) | 3 | ~40 min | CPU |
| Interference | 14 (7 batch × 2 mode) | 3 | ~40 min | CPU |
| YOLO Phase 1 | 1 baseline | 3 | ~15 min | GPU |
| YOLO Phase 3 | 4 (2 sync × 2 thread) | 3 | ~30 min | GPU |
| YOLO Phase 4 | 16 (4 batch × 2 sync × 2 thread) | 3 | ~3 hrs | GPU |

**2. Monte Carlo Experiment**

- Purpose: Evaluate batch size scaling and threading impact
- Parameters: batch sizes (1024, 2048, 4096, 8192, 16384, 32768, 65536), reactive/proactive, single/multi-threaded
- Metrics: iterations per batch, time per batch, throughput (iterations/sec), cancellation delay
- Output: `experiments/monte_carlo/results/` (CSV, JSON, plots)
- Paper reference: Section VIII, Figures 5a-5b

**3. Interference Experiment**

- Purpose: Measure timing interference between batch processing and periodic timer (100ms period, 10ms execution)
- Parameters: Same batch sizes, reactive/proactive, single-threaded only
- Metrics: timer period jitter, missed periods (>150% expected), compute batch timing
- Output: `experiments/interference/results/` (CSV, JSON, plots)
- Paper reference: Section IX-A, Figure 6

**4. YOLO Experiment**

- Purpose: Evaluate anytime YOLO with quality-based cancellation
- 7-phase workflow documented in `experiments/yolo/WORKFLOW.md`
- Phase 1: Layer-wise quality progression (which layers matter most)
- Phase 3: Maximum throughput across sync/async and threading configs
- Phase 4: Cancellation performance — how quickly can we stop and return a good result
- Output: `experiments/yolo/results/` (quality plots, throughput comparison, cancellation delay analysis)
- Paper reference: Section IX-B, Figures 7a-7b

**5. Mapping to Paper Figures**

| Paper Figure | Experiment | Analysis Script |
| ---- | ---- | ---- |
| Figure 5a: Cancellation delay vs batch size | Monte Carlo | `evaluate_monte_carlo.py` |
| Figure 5b: Throughput vs batch size | Monte Carlo | `evaluate_monte_carlo.py` |
| Figure 6: Timer interference | Interference | `evaluate_interference.py` |
| Figure 7a: Quality progression by layer | YOLO Phase 1 | `2a_analyze_quality.py` |
| Figure 7b: Cancellation performance | YOLO Phase 4 | `7_analyze_cancellation.py` |

---

## 9.3 Component Documentation

### New File: `docs/COMPONENTS.md`

#### Contents

**1. anytime_core**

- Template parameters: `InterfaceType` (action definition), `GoalHandleType`
- Virtual interface contract: `compute_single_iteration()`, `should_finish()`, `populate_result()`, `populate_feedback()`, `reset_domain_state()`, `get_batch_iterations()`, `process_gpu_completions()`
- `AnytimeWaitable`: Custom RCL waitable using `rcl_guard_condition_t`, implements `is_ready()`, `execute()`, `take_data()`
- `AnytimeServer`: Action server wrapper with goal acceptance, cancellation, and result publishing
- `AnytimeClientBase`: Action client with feedback/result processing

**2. anytime_yolo**

- `yolo.hpp`: TensorRT wrapper — `InferenceState` for layer-by-layer processing, FP16/FP32 support, detection parsing
- `anytime_management.hpp`: YOLO-specific anytime logic — 4 template variants (reactive/proactive × sync/async)
- GPU synchronization: old approach (direct callback state access) vs new approach (atomic signals + CUDA events)
- Early exit: detection quality checked per-layer, cancellation when score threshold met
- Weight files: 32 layer weights in `weights_32/` directory

**3. anytime_monte_carlo**

- Simple stochastic computation: random points in unit square, count inside unit circle, estimate pi
- `compute_single_iteration()`: One random point per iteration
- `should_finish()`: `loop_count_ >= goal` (goal = target iterations)
- Configurable seed for reproducibility

**4. anytime_tracing**

- LTTng tracepoint definitions using `TRACEPOINT_EVENT` macros
- Conditional compilation: `ANYTIME_TRACING_DISABLED` flag
- 40+ tracepoints across core, server, client, Monte Carlo, YOLO, and interference
- Adding new tracepoints: define in `tp_call.h`, declare in `anytime_tracetools.h`

**5. interference**

- Timer node with configurable period (100ms) and execution time (10ms busy-wait)
- Measures timer jitter to quantify how batch processing interferes with periodic tasks
- Uses `rclcpp::TimerBase` with LTTng instrumentation

**6. video_publisher**

- Publishes video frames or static images as `sensor_msgs/Image`
- Used as input source for YOLO experiments
- Configurable image path and publish rate

---

## Verification

```bash
# Check all markdown files render correctly
# (visual inspection in VS Code / GitHub preview)

# Verify all file paths referenced in docs exist
grep -oP '\`[a-zA-Z_/]+\.(hpp|cpp|py|sh|yaml)\`' docs/*.md | \
  while read -r ref; do
    file=$(echo "$ref" | tr -d '`')
    [ -f "$file" ] || echo "MISSING: $file"
  done
```
