# 01 — Fix CUDA Async Race Conditions

**Day:** 1 (Tue Feb 24)
**Priority:** CRITICAL — everything else depends on correct async behavior
**Estimated time:** 4-6 hours

---

## Problem

The current async GPU mode has 4 race conditions identified in `plan.md`:

1. **Non-atomic `processed_layers_++`** — `processed_layers_` is a plain `int` accessed from the CUDA callback thread without synchronization
2. **Unsafe `goal_handle_` access** — `goal_handle_->is_canceling()` and `goal_handle_->is_executing()` are called from the CUDA thread
3. **Guard condition signal collapse** — Multiple rapid `trigger()` calls collapse into one wakeup (binary semaphore, not counter)
4. **`cudaLaunchHostFunc` called with nullptr** — In sync mode, the callback is nullptr but `cudaLaunchHostFunc` is called unconditionally

---

## Files to Modify

### 1. `packages/src/anytime_yolo/include/anytime_yolo/anytime_management.hpp`

This is the core fix. Changes:

**Add new member variables:**

```cpp
// Submission tracking (executor thread only — no synchronization needed)
int layers_submitted_ = 0;
int layers_processed_ = 0;

// Completion signaling (CUDA thread -> executor thread)
std::atomic<int> completion_signals_{0};

// Event queue for verification
struct LayerEvent {
  cudaEvent_t event;
  std::atomic<bool> pending{false};
};
std::vector<LayerEvent> event_queue_;
```

**Update constructor** — Initialize event queue:

```cpp
event_queue_.resize(MAX_NETWORK_LAYERS);  // 25 for YOLO
for (auto& le : event_queue_) {
  cudaEventCreate(&le.event);
  le.pending.store(false);
}
```

**Update destructor** — Destroy events:

```cpp
for (auto& le : event_queue_) {
  cudaEventDestroy(le.event);
}
```

**Rewrite `forward_finished_callback()`** — Minimal, thread-safe:

```cpp
static void CUDART_CB forward_finished_callback(void* userData) {
  auto* self = static_cast<AnytimeManagement*>(userData);
  // ONLY two operations — both thread-safe
  self->completion_signals_.fetch_add(1, std::memory_order_release);
  self->notify_waitable();
  // NO other state access — no logging, no goal_handle_, no counters
}
```

**Modify `compute_single_iteration()` async path:**

```cpp
if constexpr (isSyncAsync) {
  int idx = layers_submitted_ % MAX_NETWORK_LAYERS;
  yolo_.inferStep(*yolo_state_, true, forward_finished_callback, this);
  cudaEventRecord(event_queue_[idx].event, stream);
  event_queue_[idx].pending.store(true, std::memory_order_release);
  layers_submitted_++;
} else {
  // Sync mode unchanged
  yolo_.inferStep(*yolo_state_, false, nullptr, nullptr);
  cudaStreamSynchronize(stream);
  processed_layers_++;
  TRACE_YOLO_LAYER_END(this->node_, processed_layers_);
  this->send_feedback();
}
```

**Add `process_gpu_completions()` method** (runs on executor thread only):

```cpp
void process_gpu_completions() override {
  int signals = completion_signals_.exchange(0, std::memory_order_acquire);
  if (signals == 0 && layers_processed_ >= layers_submitted_) return;

  int newly_completed = 0;
  while (layers_processed_ < layers_submitted_) {
    int idx = layers_processed_ % MAX_NETWORK_LAYERS;
    if (!event_queue_[idx].pending.load(std::memory_order_acquire)) break;

    cudaError_t status = cudaEventQuery(event_queue_[idx].event);
    if (status == cudaSuccess) {
      event_queue_[idx].pending.store(false, std::memory_order_release);
      layers_processed_++;
      processed_layers_++;
      newly_completed++;
      TRACE_YOLO_LAYER_END(this->node_, processed_layers_);
      this->send_feedback();
    } else if (status == cudaErrorNotReady) {
      break;
    } else {
      RCLCPP_ERROR(this->node_->get_logger(),
                   "CUDA error querying event %d: %s",
                   layers_processed_, cudaGetErrorString(status));
      break;
    }
  }

  // Re-trigger if more work pending
  if (layers_processed_ < layers_submitted_) {
    this->notify_waitable();
  }
}
```

**Update `reset_domain_state()`** — Reset new counters and event queue:

```cpp
layers_submitted_ = 0;
layers_processed_ = 0;
completion_signals_.store(0);
for (auto& le : event_queue_) {
  le.pending.store(false);
}
```

### 2. `packages/src/anytime_yolo/include/anytime_yolo/yolo.hpp` (~line 1067)

Guard the `cudaLaunchHostFunc` call:

```cpp
// Before (bug):
cudaLaunchHostFunc(stream, callback, userData);

// After (fix):
if (callback != nullptr) {
  cudaLaunchHostFunc(stream, callback, userData);
}
```

### 3. `packages/src/anytime_core/include/anytime_core/anytime_base.hpp`

Add virtual hook for GPU completion processing:

```cpp
// Add as protected virtual method
virtual void process_gpu_completions() {}
```

Call it in `reactive_anytime_function()` and `proactive_anytime_function()` before `compute()`:

```cpp
void reactive_anytime_function() {
  process_gpu_completions();  // Drain any pending GPU work
  // ... existing compute logic
}
```

### 4. `packages/src/anytime_core/include/anytime_core/anytime_waitable.hpp` (optional)

Consider integrating GPU completion callback into the `execute()` flow as an alternative integration point. The simpler approach is via `anytime_base.hpp` as described above.

---

## Safety Properties Guaranteed

- **No Lost Completions:** Atomic counter + exhaustive event checking
- **No Double-Processing:** Submission-order tracking with `layers_processed_` counter
- **Spurious Wakeup Safe:** Event checking is idempotent
- **Thread-Safe:** Proper memory ordering (acquire/release semantics)
- **Level-Triggered:** Guard condition stays triggered if new work arrives during processing

---

## Fallback

If the full event-based approach has issues, use the simpler counter-only approach from `plan.md` (lines 287-306). It uses only atomics without CUDA events — still correct, just less verifiable.

---

## Verification

```bash
# 1. Build
colcon build --packages-select anytime_yolo

# 2. Test sync mode (regression check)
ros2 launch experiments yolo.launch.py \
  server_config:=phase1_baseline_server.yaml \
  client_config:=phase1_baseline_client.yaml

# 3. Test async mode with DEBUG logging
ros2 run anytime_yolo anytime_server --ros-args \
  -p is_sync_async:=async \
  -p is_reactive_proactive:=reactive \
  --log-level DEBUG

# 4. Check logs for:
#    - "GPU completion check:" messages
#    - No "processed_layers" mismatches
#    - All layers accounted for
```
