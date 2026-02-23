# 04 — Integration Tests (GPU Sync + Cancellation)

**Day:** 2 (Wed Feb 25)
**Priority:** MEDIUM
**Estimated time:** 2-3 hours
**Depends on:** 01 (GPU sync fix)
**Requires:** GPU hardware (CUDA)

---

## Overview

Integration tests that validate the GPU synchronization fix and cancellation behavior. These require CUDA-capable hardware and should gracefully skip on CPU-only environments.

---

## 4.1 GPU Sync Tests

### New File: `packages/src/anytime_yolo/test/test_gpu_sync.cpp`

All tests should be guarded:

```cpp
#ifdef CUDA_AVAILABLE
// test body
#else
GTEST_SKIP() << "CUDA not available";
#endif
```

| Test | Description |
| ---- | ---- |
| `test_sync_mode_completes_all_layers` | Run sync mode inference, verify `processed_layers == 25` (all YOLO layers) |
| `test_async_single_layer_completion` | Submit 1 layer in async mode, wait for completion, verify 1 event processed |
| `test_async_all_layers_complete` | Submit all 25 layers async, verify all 25 processed |
| `test_sync_async_output_equivalence` | Same input image through both modes, verify identical detection outputs |
| `test_event_queue_circular_reuse` | Submit `MAX_NETWORK_LAYERS + 5` layers, verify circular buffer indices work correctly |
| `test_completion_signals_atomic` | Verify `completion_signals_` counter matches number of completed layers |
| `test_no_lost_completions_under_load` | Rapid submission of layers, verify all completions tracked |

### Key Test Fixture

```cpp
class GpuSyncTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    // Create node, load YOLO weights, initialize management
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  // Helper to run N layers and verify N completions
  void run_and_verify(int num_layers, bool async);
};
```

---

## 4.2 Cancellation Tests

### New File: `packages/src/anytime_yolo/test/test_cancellation.cpp`

| Test | Description |
| ---- | ---- |
| `test_cancel_during_gpu_work` | Submit layers, cancel after 3, verify clean cancellation with intermediate result |
| `test_no_crash_callback_after_cancel` | Cancel, then ensure callbacks arriving post-cancel don't segfault |
| `test_quality_based_cancellation` | Monitor feedback, cancel when score threshold met, verify result quality |
| `test_cancel_returns_partial_result` | Cancel at layer 10, verify result contains detections from layers 1-10 |
| `test_reset_after_cancel` | Cancel, then start new goal, verify clean state |

---

## 4.3 Build Configuration

### Modify: `packages/src/anytime_yolo/CMakeLists.txt`

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Check for CUDA
  find_package(CUDA QUIET)
  if(CUDA_FOUND)
    add_definitions(-DCUDA_AVAILABLE)

    ament_add_gtest(test_gpu_sync test/test_gpu_sync.cpp)
    target_link_libraries(test_gpu_sync anytime_yolo_lib ${CUDA_LIBRARIES})
    ament_target_dependencies(test_gpu_sync rclcpp rclcpp_action)

    ament_add_gtest(test_cancellation test/test_cancellation.cpp)
    target_link_libraries(test_cancellation anytime_yolo_lib ${CUDA_LIBRARIES})
    ament_target_dependencies(test_cancellation rclcpp rclcpp_action)
  endif()
endif()
```

---

## Verification

```bash
# Build with tests (GPU environment)
colcon build --packages-select anytime_yolo

# Run GPU tests
colcon test --packages-select anytime_yolo

# Run with ThreadSanitizer (if available)
colcon build --packages-select anytime_yolo \
  --cmake-args -DCMAKE_CXX_FLAGS="-fsanitize=thread"
colcon test --packages-select anytime_yolo

# View results
colcon test-result --verbose
```

---

## Notes

- These tests need YOLO weights at the configured path
- Test images should be included in the test fixtures
- On CPU-only CI, these tests will be skipped automatically via `GTEST_SKIP()`
- ThreadSanitizer is the primary tool for detecting remaining race conditions
