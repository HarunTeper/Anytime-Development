GPU Synchronization for Anytime ROS 2 - Cooperative Notification Mechanism
Context
The Anytime ROS 2 framework (paper: main.pdf) requires a synchronization mechanism between the ROS 2 executor and GPU computation that supports both synchronous and asynchronous execution modes.

Current Implementation:

Sync mode: Uses cudaStreamSynchronize (blocking) - works correctly ✅
Async mode: Uses cudaLaunchHostFunc with static callback - has thread safety issues ⚠️
Problems with Current Async Approach:

Thread safety: processed_layers_++ is non-atomic (race condition)
Unsafe void pointer casting: reinterpret_cast<void*>(&forward_finished_callback) violates strict aliasing
Shared pointer access: goal_handle_ accessed from CUDA callback thread without synchronization
Complex cross-thread debugging: Hard to trace execution across CPU and GPU threads
Why We Need Cooperative Notification:

Per paper (Section VI-B, page 7): Need to notify executor when GPU layer completes
Passive polling mechanism performs worse than cooperative approach (paper validation)
Must maintain ROS 2 single-threaded, non-preemptive executor semantics
Must integrate with existing guard condition architecture (Figure 4a in paper)
Problem Definition
Core Challenge
Synchronize GPU layer completions with ROS 2's single-threaded executor without:

Blocking the executor (prevents cancellation)
Using polling (performance overhead)
Losing completion notifications (correctness)
Thread safety violations (undefined behavior)
Critical Race Conditions Identified
Race 1: Guard Condition Signal Collapse
Problem: Multiple GPU layers complete rapidly → multiple trigger() calls → only ONE executor wake-up


T0: Layer 1 completes → trigger() → guard_condition = TRIGGERED
T1: Layer 2 completes → trigger() → guard_condition = TRIGGERED (no change)
T2: Layer 3 completes → trigger() → guard_condition = TRIGGERED (no change)
T3: Executor wakes → processes only 1 layer
T4: Layers 2-3 notifications LOST ❌
Root Cause: Guard condition is a binary semaphore, not a counter.

Race 2: Executor Processing While New Completions Arrive
Problem: Reading and resetting counter is not atomic


Executor: completed = pending_.load()  // Reads 2
CUDA:                                    pending_++ (now = 3)
Executor: pending_.store(0)            // Resets, loses layer 3!
Race 3: Event Reuse in Circular Buffer
Problem: Reusing CUDA events can confuse old completions with new submissions

Race 4: Memory Ordering
Problem: Without proper memory barriers, executor may see stale data from CUDA thread

Solution: Cooperative Notification with Event-Based Verification
Key Insight
The atomic counter is a hint (wake-up signal), CUDA events are the source of truth (actual state).

Architecture

┌─────────────────┐           ┌──────────────────┐
│   GPU Thread    │           │ Executor Thread  │
│  (CUDA Stream)  │           │   (ROS 2)        │
└────────┬────────┘           └────────┬─────────┘
         │                             │
         │ Layer completes             │
         │ (async callback)            │
         │                             │
         ├─1─ pending_.fetch_add(1) ──┤  Atomic counter
         │                             │  (thread-safe signal)
         │                             │
         ├─2─ notify_waitable() ──────┤  Trigger guard condition
         │    (trigger)                │  (wake up executor)
         │                             │
         │                             ▼
         │                      is_ready() returns true
         │                             │
         │                             ▼
         │                      execute() called
         │                             │
         │                             ├─1─ pending_.exchange(0)
         │                             │    (atomic read & reset)
         │                             │
         │                             ├─2─ Check ALL cudaEvents
         │                             │    (source of truth)
         │                             │
         │                             ├─3─ Process completed layers
         │                             │    send_feedback()
         │                             │
         │                             ▼
         │                      Continue or finish
Safety Properties Guaranteed
✅ No Lost Completions: Atomic counter + exhaustive event checking
✅ No Double-Processing: Submission-order tracking with processed_ counter
✅ Spurious Wakeup Safe: Event checking is idempotent
✅ Thread-Safe: Proper memory ordering (acquire/release semantics)
✅ Level-Triggered: Guard condition stays triggered if new work arrives during processing

Implementation Plan
Files to Modify
Primary Changes:
packages/src/anytime_yolo/include/anytime_yolo/anytime_management.hpp

Add event queue and atomic counters
Modify compute_single_iteration() for event recording
Rewrite forward_finished_callback() to use atomic signals only
Add process_gpu_completions() method
packages/src/anytime_core/include/anytime_core/anytime_waitable.hpp

Add GPU completion callback registration
Integrate with existing guard condition mechanism
packages/src/anytime_core/include/anytime_core/anytime_base.hpp

Add virtual process_gpu_completions() method (optional)
Ensure waitable can call GPU completion handler
Implementation Details
Step 1: Add Data Structures (anytime_management.hpp)

template <bool isReactiveProactive, bool isSyncAsync>
class AnytimeManagement : public anytime_core::AnytimeBase<...> {
private:
  // Submission tracking (executor thread only)
  int layers_submitted_ = 0;
  int layers_processed_ = 0;

  // Completion signaling (CUDA thread → executor)
  std::atomic<int> completion_signals_{0};

  // Event queue for verification
  struct LayerEvent {
    cudaEvent_t event;
    std::atomic<bool> pending{false};
  };
  std::vector<LayerEvent> event_queue_;

public:
  AnytimeManagement(...) : ... {
    // Initialize event queue
    event_queue_.resize(MAX_NETWORK_LAYERS);
    for (auto& le : event_queue_) {
      cudaEventCreate(&le.event);
      le.pending.store(false);
    }
  }

  ~AnytimeManagement() {
    for (auto& le : event_queue_) {
      cudaEventDestroy(le.event);
    }
  }
};
Step 2: Modify compute_single_iteration()

void compute_single_iteration() override {
  RCLCPP_DEBUG(this->node_->get_logger(), "YOLO compute single iteration called");
  TRACE_YOLO_LAYER_START(this->node_, processed_layers_);

  if constexpr (isSyncAsync) {
    // ASYNC MODE: Record event, use callback
    int idx = layers_submitted_ % MAX_NETWORK_LAYERS;

    // Launch GPU work with callback
    yolo_.inferStep(*yolo_state_, true, forward_finished_callback, this);

    // Record completion event AFTER the inference step
    cudaEventRecord(event_queue_[idx].event, stream);
    event_queue_[idx].pending.store(true, std::memory_order_release);

    layers_submitted_++;

    RCLCPP_DEBUG(this->node_->get_logger(),
                 "Async layer submitted: %d", layers_submitted_);

  } else {
    // SYNC MODE: Simple blocking (unchanged)
    yolo_.inferStep(*yolo_state_, false, nullptr, nullptr);
    cudaStreamSynchronize(stream);

    processed_layers_++;
    TRACE_YOLO_LAYER_END(this->node_, processed_layers_);
    RCLCPP_DEBUG(this->node_->get_logger(),
                 "Sync layer processed: %d", processed_layers_);
    this->send_feedback();
  }
}
Step 3: Rewrite CUDA Callback (Thread-Safe)

static void CUDART_CB forward_finished_callback(void* userData) {
  auto* this_ptr = static_cast<AnytimeManagement*>(userData);

  // ONLY signal completion - don't access any other state
  // This is the ONLY operation in callback (thread-safe)
  this_ptr->completion_signals_.fetch_add(1, std::memory_order_release);

  // Trigger guard condition to wake executor (thread-safe operation in ROS 2)
  this_ptr->notify_waitable();

  // NO other operations - no logging, no state access, no incrementing counters
  // All processing happens on executor thread
}
Critical: The callback only does TWO things:

Atomic increment of signal counter
Trigger guard condition
No other state access → no race conditions!

Step 4: Add GPU Completion Processing (Executor Thread)

void process_gpu_completions() {
  // Read and reset signal counter atomically
  int signals = completion_signals_.exchange(0, std::memory_order_acquire);

  RCLCPP_DEBUG(this->node_->get_logger(),
               "GPU completion check: signals=%d, submitted=%d, processed=%d",
               signals, layers_submitted_, layers_processed_);

  // Check all pending events (even if signals=0, handles spurious wakeups)
  int newly_completed = 0;

  while (layers_processed_ < layers_submitted_) {
    int idx = layers_processed_ % MAX_NETWORK_LAYERS;

    // Check if this event is marked as pending
    if (!event_queue_[idx].pending.load(std::memory_order_acquire)) {
      break;  // Not submitted yet or already processed
    }

    // Query CUDA event status
    cudaError_t status = cudaEventQuery(event_queue_[idx].event);

    if (status == cudaSuccess) {
      // Layer completed!
      event_queue_[idx].pending.store(false, std::memory_order_release);
      layers_processed_++;
      newly_completed++;

      // Update domain state
      processed_layers_++;
      TRACE_YOLO_LAYER_END(this->node_, processed_layers_);

      // Send feedback for this layer
      this->send_feedback();

      RCLCPP_DEBUG(this->node_->get_logger(),
                   "Layer %d completed", layers_processed_);

    } else if (status == cudaErrorNotReady) {
      // This layer not ready yet, stop checking
      break;

    } else {
      // CUDA error occurred
      RCLCPP_ERROR(this->node_->get_logger(),
                   "CUDA error querying event %d: %s",
                   layers_processed_, cudaGetErrorString(status));
      break;
    }
  }

  if (newly_completed > 0) {
    RCLCPP_DEBUG(this->node_->get_logger(),
                 "Processed %d GPU completions (signals=%d)",
                 newly_completed, signals);
  }

  // Trigger next iteration if more work needed
  if (layers_processed_ < layers_submitted_ || !should_finish()) {
    this->notify_waitable();
  }
}
Step 5: Integrate with AnytimeWaitable
Modify execute flow in waitable to call GPU completion handler:


// In anytime_waitable.hpp or where execute is called
void execute() override {
  // First, process any completed GPU work
  if constexpr (isSyncAsync) {
    this->process_gpu_completions();
  }

  // Then continue with regular anytime algorithm flow
  execute_callback_();
}
Or, add to the anytime_base reactive/proactive functions to check GPU completions before each iteration.

Alternative: Simpler Counter-Only Approach (Less Safe)
If event checking overhead is too high, can use counter-only approach with atomics:


// Simpler but requires trusting callback count
std::atomic<int> pending_completions_{0};

static void CUDART_CB callback(void* userData) {
  auto* m = static_cast<AnytimeManagement*>(userData);
  m->pending_completions_.fetch_add(1, std::memory_order_release);
  m->notify_waitable();
}

void process_gpu_completions() {
  int completed = pending_completions_.exchange(0, std::memory_order_acquire);

  for (int i = 0; i < completed; i++) {
    processed_layers_++;
    send_feedback();
  }
}
Trade-off: Simpler but less robust (trusts callback count, no verification against CUDA state)

Recommendation: Use full event-based approach for correctness, optimize later if needed.

Testing & Verification
Unit Tests to Add
Test single layer completion

Submit 1 layer → verify 1 completion processed
Test multiple rapid completions

Submit 5 layers → verify all 5 processed correctly
Check no completions lost
Test spurious wakeup handling

Trigger guard condition manually → verify no crash
Test event reuse

Submit MAX_NETWORK_LAYERS + 5 → verify circular buffer works
Test sync vs async equivalence

Run same input in sync and async modes → verify same outputs
Integration Tests
Cancellation during GPU work

Submit layers → cancel after 3 layers → verify clean cancellation
Check no segfaults from callback accessing freed memory
Quality-based cancellation (from paper)

Monitor feedback quality → cancel when threshold met
Verify correct intermediate result returned
Stress test

Run with batch_size=1 (many callbacks)
Run with batch_size=25 (fewer callbacks)
Verify no race conditions under load
Manual Verification

# 1. Build with async mode
cd /home/daes-enzo/Anytime-Development/packages
colcon build --packages-select anytime_yolo

# 2. Run with logging
ros2 run anytime_yolo anytime_server --ros-args \
  -p is_sync_async:=async \
  -p is_reactive_proactive:=reactive \
  --log-level DEBUG

# 3. Check logs for:
# - "GPU completion check:" messages
# - No "processed_layers" mismatches
# - All layers accounted for

# 4. Run experiments from paper (Section IX)
ros2 launch experiments yolo.launch.py

# 5. Verify metrics match paper results
# - Cancellation delay (Figure 5b)
# - Total runtime (Figure 7b)
Rollback Plan
If issues arise, can revert to synchronous mode:

Set is_sync_async: "sync" in config files
Remove async-specific code (keep behind if constexpr (isSyncAsync))
Synchronous mode works correctly already (no changes needed)
Performance Considerations
Expected Overhead
Event creation: One-time cost at initialization (negligible)
Event recording: ~1-5μs per layer (minimal)
Event querying: ~0.5μs per check (cheap)
Atomic operations: ~10-50ns (negligible)
Optimization Opportunities (Future)
Batch event checking: Check N events per wake-up
Event pool: Reuse events more efficiently
Lock-free queue: Use boost::lockfree for completion queue
CUDA graphs: For fixed inference patterns (not applicable to anytime algorithms)
Open Questions
Integration point: Where exactly to call process_gpu_completions()?

Option A: In AnytimeWaitable::execute()
Option B: At start of reactive/proactive anytime functions
Recommendation: Option A (cleaner separation)
Error handling: What if cudaEventQuery returns error?

Current plan: Log error and stop processing
Alternative: Continue checking other events?
Batch size interaction: How does this interact with batch_size > 1?

Currently: Each layer gets its own event
Alternative: One event per batch?
Recommendation: Keep per-layer for fine-grained cancellation
References
Paper: /home/daes-enzo/Anytime-Development/main.pdf
Section VI-B: Accelerator synchronization mechanisms (page 7)
Section IX-C: AnytimeYOLO evaluation (page 10)
Figure 4: Cooperative vs Passive notification mechanisms
Current implementation: packages/src/anytime_yolo/include/anytime_yolo/anytime_management.hpp
CUDA documentation: cudaEventQuery, cudaEventRecord, cudaLaunchHostFunc
ROS 2 guard conditions: rclcpp/guard_condition.hpp
Success Criteria
✅ Async mode achieves same correctness as sync mode
✅ All layers processed (no lost completions)
✅ No race conditions (ThreadSanitizer clean)
✅ No memory errors (Valgrind/AddressSanitizer clean)
✅ Cancellation works correctly (can cancel mid-processing)
✅ Performance matches paper results (minimal overhead vs current implementation)