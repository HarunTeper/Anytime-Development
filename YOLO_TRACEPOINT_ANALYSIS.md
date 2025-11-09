# YOLO Tracepoint Analysis

## Date: November 9, 2025

## Summary
Analysis of YOLO tracepoint implementation to ensure all required events are traced for experiments.

---

## 1. Defined Tracepoints

### Location: `packages/src/anytime_tracing/include/anytime_tracing/anytime_tracetools.h`

All YOLO-specific tracepoints are defined:

1. **`yolo_init`** - YOLO initialization
   - Parameters: node_handle, batch_size, is_reactive_proactive, is_sync_async, weights_path
   - Status: ✅ DEFINED and IMPLEMENTED

2. **`yolo_layer_start`** - Layer processing start
   - Parameters: node_handle, layer_num
   - Status: ✅ DEFINED and IMPLEMENTED

3. **`yolo_layer_end`** - Layer processing end
   - Parameters: node_handle, layer_num
   - Status: ✅ DEFINED and IMPLEMENTED

4. **`yolo_exit_calculation_start`** - Exit calculation start
   - Parameters: node_handle, layer_num
   - Status: ✅ DEFINED and **NOW IMPLEMENTED**

5. **`yolo_exit_calculation_end`** - Exit calculation end with detection count
   - Parameters: node_handle, layer_num, num_detections
   - Status: ✅ DEFINED and **NOW IMPLEMENTED** (CRITICAL for quality analysis)

6. **`yolo_detection`** - Individual detection details
   - Parameters: node_handle, layer_num, detection_id, confidence, class_id, bbox_x, bbox_y, bbox_width, bbox_height
   - Status: ✅ DEFINED and **NOW IMPLEMENTED**

7. **`yolo_result`** - Final result summary
   - Parameters: node_handle, processed_layers, result_processed_layers, total_detections
   - Status: ✅ DEFINED and IMPLEMENTED

8. **`yolo_reset`** - Domain state reset
   - Parameters: node_handle
   - Status: ✅ DEFINED and IMPLEMENTED

9. **`yolo_image_processed`** - Image processing info
   - Parameters: node_handle, image_width, image_height
   - Status: ✅ DEFINED and IMPLEMENTED

10. **`yolo_cuda_callback`** - CUDA callback notification
    - Parameters: node_handle, processed_layers
    - Status: ✅ DEFINED and IMPLEMENTED

---

## 2. Implementation Status

### Currently Implemented in `anytime_management.hpp`:

| Tracepoint | Called In | Line/Function |
|------------|-----------|---------------|
| `TRACE_YOLO_INIT` | Constructor | Line 37 |
| `TRACE_YOLO_LAYER_START` | compute_single_iteration() | Line 47 |
| `TRACE_YOLO_LAYER_END` | compute_single_iteration() | Line 62 |
| `TRACE_YOLO_RESULT` | populate_result() | Line 168 |
| `TRACE_YOLO_RESET` | reset_domain_state() | Line 175 |
| `TRACE_YOLO_IMAGE_PROCESSED` | reset_domain_state() | Line 189 |
| `TRACE_YOLO_CUDA_CALLBACK` | forward_finished_callback() | Line 291 |

### ✅ Now Implemented (as of Nov 9, 2025):

| Tracepoint | Implemented In | Purpose |
|------------|----------------|---------|
| `TRACE_YOLO_EXIT_CALCULATION_START` | `anytime_management.hpp::populate_result()` line 108 | Mark when exit calculation begins |
| `TRACE_YOLO_EXIT_CALCULATION_END` | `anytime_management.hpp::populate_result()` line 127 | **CRITICAL**: Record detection count per layer |
| `TRACE_YOLO_DETECTION` | `anytime_management.hpp::populate_result()` line 173-176 | Detailed detection info with bbox coordinates |

---

## 3. Impact on Experiments

### Phase 1: Baseline (Current)
**Status**: ⚠️ **PARTIALLY WORKING**

The analysis script `analyze_quality.py` **requires** `yolo_exit_calculation_end` events:
```python
elif event.event_name == 'yolo_exit_calculation_end':
    # Detection count after exit calculation at this layer
    layer_num = event.fields.get('layer_num', 0)
    num_detections = event.fields.get('num_detections', 0)
    current_image['layer_detections'][layer_num] = num_detections
```

**Without this event:**
- ❌ Cannot track layer-wise detection progression
- ❌ Cannot calculate quality ratios (intermediate/final detections)
- ❌ Cannot determine optimal cancellation thresholds
- ❌ Phase 2 quality analysis will be incomplete

### Phase 2: Quality Analysis
**Status**: ❌ **BLOCKED**

The entire Phase 2 analysis depends on having detection counts at each layer to:
1. Track how detection quality improves with each layer
2. Calculate quality ratios (detections at layer N / final detections)
3. Determine 90%, 95%, 99% quality thresholds
4. Generate cancellation point recommendations

### Phase 3 & 4: Throughput and Full Sweep
**Status**: ⚠️ **AFFECTED**

Without proper exit calculation tracing:
- Cannot correlate throughput with quality
- Cannot validate if early exits maintain acceptable quality
- Limited insight into when cancellations preserve accuracy

---

## 4. Current Baseline Run Analysis

Based on terminal output showing user ran Phase 1:
```bash
babeltrace /home/vscode/workspace/experiments/yolo/traces/phase1_baseline_trial1 | grep 'anytime_base_activate' | wc -l
```

The traces exist but are missing critical `yolo_exit_calculation_end` events needed for quality analysis.

---

## 5. Recommended Fixes

### Priority 1: CRITICAL - Add Exit Calculation Tracing

**Location**: `packages/src/anytime_yolo/include/anytime_yolo/yolo.hpp`

#### Fix A: In `calculateLatestExit()` function (around line 1146)

The function already computes detections via `processNMSAndGetResults()`, but needs to:
1. Count detections in the result
2. Emit tracepoint with layer number and detection count

**Where to add**: After `processNMSAndGetResults()` returns, before returning result.

#### Fix B: Alternative - In `anytime_management.hpp::populate_result()`

Since this function already processes the YOLO results and has access to:
- `processed_layers_` - which layer we're at
- The detection vector from `yolo_result`

Could add tracepoint here to record layer-wise detection counts.

**Challenge**: Need to determine which layer produced each detection.

#### Recommendation: Modify `yolo.hpp`

Better to trace in `calculateLatestExit()` and `finishEarly()` because:
- These functions know exactly which exit/layer is being calculated
- They have the detection results immediately after NMS
- More accurate layer attribution

### Priority 2: OPTIONAL - Add Detection-Level Tracing

**Location**: `packages/src/anytime_yolo/include/anytime_yolo/anytime_management.hpp`

In `populate_result()` function, after line 115 where detections are processed:
```cpp
// Add detection to result
result->detections.push_back(detection);

// OPTIONAL: Add tracepoint for each detection
TRACE_YOLO_DETECTION(
    this->node_, processed_layers_, i/6, 
    yolo_result[i + 4], static_cast<int>(yolo_result[i + 5]),
    yolo_result[i], yolo_result[i + 1], 
    yolo_result[i + 2] - yolo_result[i], 
    yolo_result[i + 3] - yolo_result[i + 1]
);
```

This would provide fine-grained detection tracking but may generate large trace files.

---

## 6. Implementation Details Needed

To properly implement `yolo_exit_calculation_end`, need to understand:

### Question 1: When does exit calculation happen?
- In **Proactive mode**: `calculateLatestExit()` is called in `populate_result()`
- In **Reactive mode**: `finishEarly()` is called in `populate_result()`
- Both eventually call `processNMSAndGetResults()` to get detections

### Question 2: Which layer should be attributed?
Looking at the code:
- `calculateLatestExit()` finds the "best exit" based on `currentIndex` (current layer)
- `finishEarly()` also calls `calculateLatestExit()`
- The exit is determined by which subexits have inputs ready

The `processed_layers_` variable in `anytime_management.hpp` tracks this.

### Question 3: Where to inject tracepoint?

**Option A: In `yolo.hpp` functions**
```cpp
// In calculateLatestExit() or finishEarly()
auto result = processNMSAndGetResults(input);

// Count non-zero detections
int detection_count = 0;
for (size_t i = 0; i < result.size(); i += 6) {
    if (result[i + 4] > 0.0) { // confidence > 0
        detection_count++;
    }
}

// Need to pass node handle somehow...
// TRACE_YOLO_EXIT_CALCULATION_END(node, state.currentIndex, detection_count);
```

**Challenge**: These functions in `yolo.hpp` don't have access to the ROS node handle!

**Option B: In `anytime_management.hpp::populate_result()`**
```cpp
// After calling calculateLatestExit or finishEarly
yolo_result = yolo_.calculateLatestExit(*yolo_state_);

// Count detections
int detection_count = 0;
for (size_t i = 0; i < yolo_result.size(); i += 6) {
    if (yolo_result[i + 4] > 0.0) {
        detection_count++;
    }
}

// Emit tracepoint BEFORE processing individual detections
TRACE_YOLO_EXIT_CALCULATION_END(this->node_, processed_layers_, detection_count);

// Then process detections as normal...
for (size_t i = 0; i < yolo_result.size(); i += 6) {
    // ... existing code ...
}
```

**Recommendation**: Use Option B - easier to implement and has all needed context.

---

## 7. Testing Requirements

After adding tracepoints:

1. **Rebuild the workspace**:
   ```bash
   cd /home/vscode/workspace/packages
   colcon build --symlink-install
   ```

2. **Re-run Phase 1 baseline**:
   ```bash
   cd /home/vscode/workspace/experiments/yolo
   ./run_phase1_baseline.sh
   ```

3. **Verify tracepoint presence**:
   ```bash
   babeltrace traces/phase1_baseline_trial1 | grep 'yolo_exit_calculation_end'
   ```

4. **Run quality analysis**:
   ```bash
   python3 analyze_quality.py
   ```

---

## 8. Conclusion

### Current Status ✅ **IMPLEMENTATION COMPLETE**
- ✅ All 10 YOLO tracepoints are now implemented and ready for use
- ✅ Exit calculation tracepoints added to `populate_result()` function
- ✅ Detection-level tracing implemented for detailed analysis
- ✅ Quality analysis can now proceed with complete data

### Implementation Summary (Nov 9, 2025)
1. ✅ **COMPLETED**: Added `TRACE_YOLO_EXIT_CALCULATION_START` at line 108
2. ✅ **COMPLETED**: Added detection counting logic (lines 120-125)
3. ✅ **COMPLETED**: Added `TRACE_YOLO_EXIT_CALCULATION_END` at line 127
4. ✅ **COMPLETED**: Added `TRACE_YOLO_DETECTION` at lines 173-176

### Next Steps
**REQUIRED** - Rebuild and test:
1. Rebuild the workspace to compile the changes
2. Re-run Phase 1 baseline experiments to generate new traces with complete data
3. Verify tracepoints in traces using babeltrace
4. Run Phase 2 quality analysis to determine optimal cancellation thresholds

### Priority
**READY FOR TESTING** - Implementation complete, now ready for rebuild and validation.

---

## Appendix: Code References

### Tracepoint Definitions
- Header: `/packages/src/anytime_tracing/include/anytime_tracing/anytime_tracetools.h`
- Implementation: `/packages/src/anytime_tracing/src/anytime_tracetools.c`
- LTTng Events: `/packages/src/anytime_tracing/include/anytime_tracing/tp_call.h`

### Usage Locations
- YOLO Management: `/packages/src/anytime_yolo/include/anytime_yolo/anytime_management.hpp`
- YOLO Core: `/packages/src/anytime_yolo/include/anytime_yolo/yolo.hpp`
- Tracing Macros: `/packages/src/anytime_yolo/include/anytime_yolo/tracing.hpp`

### Analysis Scripts
- Quality Analysis: `/experiments/yolo/analyze_quality.py`
- Phase 1 Runner: `/experiments/yolo/run_phase1_baseline.sh`
