# Step 2: Minimal-Safe GPU Sync Fix

**Status:** Pending
**Day:** 1 (Tue Feb 24)
**Priority:** CRITICAL
**Time:** ~3h
**Dependencies:** None

## Why

The async GPU mode has race conditions that cause crashes or incorrect results during YOLO experiments.

**AE Impact:** Criterion 1 (Repeatability) — YOLO experiments may fail without this.

## Three Gaps Being Fixed

1. **Stage-progress deadlock**: All 25 layers submitted → `get_batch_iterations()` returns 0 → executor hangs. EXIT/NMS/COMPLETED stages never run.
2. **Stale callback contamination**: Cancel mid-goal + new goal → in-flight CUDA callbacks from old goal corrupt new goal's count.
3. **Callback overcounting**: `cudaLaunchHostFunc` fires for EXIT/NMS/COMPLETED stages, producing fake completion signals.

## Approach

Minimal-safe counter design (no event queue). `processed_layers_` stays as plain `int` (executor-thread owned). Only `completion_signals_` is atomic (cross-thread). Callbacks are only attached for actual layer submissions.

## Changes

### File: `packages/src/anytime_core/include/anytime_core/anytime_base.hpp`

1. **Add virtual `process_gpu_completions()` hook** (default no-op)
2. **Call it TWICE** in both `reactive_anytime_function()` and `proactive_anytime_function()`:
   - Once before `compute()` (drain previous batch)
   - Once after `compute()` and before finish/cancel checks (drain current batch)
3. **Remove `get_iteration_callback()` virtual** (unused, strict aliasing violation in override)

### File: `packages/src/anytime_yolo/include/anytime_yolo/anytime_management.hpp`

4. **Add members**: `int submitted_layers_ = 0;` and `std::atomic<int> completion_signals_{0};`
5. **Rewrite `forward_finished_callback()`** to 3 lines: atomic increment + notify_waitable, nothing else
6. **Add `process_gpu_completions()` override**: drain signals, clamp to `outstanding = submitted - processed`, update `processed_layers_`
7. **Rewrite `compute_single_iteration()`**: only attach callback for `LAYER_PROCESSING` stage, increment `submitted_layers_` only then
8. **Rewrite `get_batch_iterations()`**: in async mode, return 1 when all layers submitted but not completed (drives EXIT/NMS)
9. **Update `reset_domain_state()`**: call `yolo_.synchronize()` before reset to drain stale callbacks, then reset all counters
10. **Remove `get_iteration_callback()` override** and duplicate `MAX_NETWORK_LAYERS`

### File: `packages/src/anytime_yolo/include/anytime_yolo/yolo.hpp`

11. **Guard `cudaLaunchHostFunc`** with null check, remove variable shadowing
12. **`cudaMemset` safety** — moved to Step 3

## Test Scenarios

- Async, batch_size=1: no crash/hang, `processed_layers_ == 25`
- Async, batch_size=25: no deadlock, result returns normally
- Cancel mid-goal + new goal: no stale-layer carryover
- Sync regression: behavior unchanged
- No `get_iteration_callback` references remain

## Verification

```bash
colcon build --packages-select anytime_core anytime_yolo
grep -rn "get_iteration_callback" packages/src/
colcon test --packages-select anytime_core
```
