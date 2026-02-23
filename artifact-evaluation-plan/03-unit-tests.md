# 03 — Unit Tests

**Day:** 2 (Wed Feb 25)
**Priority:** HIGH
**Estimated time:** 3-4 hours
**Depends on:** 01 (GPU sync fix)

---

## Overview

Add unit tests for the core framework and Monte Carlo implementation. These tests run on CPU-only environments and form the foundation of the test suite.

---

## 3.1 Unit Tests for anytime_core

### New File: `packages/src/anytime_core/test/test_anytime_waitable.cpp`

Tests for the guard condition / waitable mechanism:

| Test | Description |
| ---- | ---- |
| `test_trigger_makes_ready` | Create waitable, trigger it, verify `is_ready()` returns true |
| `test_execute_calls_callback` | Create waitable with callback, trigger, execute, verify callback invoked |
| `test_multiple_triggers` | Trigger multiple times, verify single wakeup (documents binary semaphore behavior) |
| `test_not_ready_initially` | Verify `is_ready()` returns false before any trigger |
| `test_null_callback_no_crash` | Create waitable without callback, execute, verify no crash |

### New File: `packages/src/anytime_core/test/test_anytime_base.cpp`

Create a `MockAnytimeBase` subclass that implements pure virtual methods for testing:

```cpp
class MockAnytimeBase : public anytime_core::AnytimeBase<TestAction, TestGoalHandle> {
  int iterations_called_ = 0;
  bool should_finish_flag_ = false;

  void compute_single_iteration() override { iterations_called_++; }
  bool should_finish() override { return should_finish_flag_; }
  void populate_feedback(Feedback& fb) override {}
  void populate_result(Result& result) override {}
  void reset_domain_state() override { iterations_called_ = 0; }
  int get_batch_iterations() override { return batch_size_; }
};
```

| Test | Description |
| ---- | ---- |
| `test_compute_calls_iteration_batch_times` | Set batch_size=5, call compute(), verify `compute_single_iteration()` called 5 times |
| `test_batch_iterations_default` | Verify `get_batch_iterations()` returns `batch_size_` |
| `test_reset_clears_domain_state` | Call reset, verify domain state cleared |
| `test_activate_sets_running` | Activate, verify `is_running()` is true |
| `test_deactivate_clears_running` | Activate then deactivate, verify `is_running()` is false |
| `test_process_gpu_completions_default_noop` | Call `process_gpu_completions()` on base, verify no crash (default no-op) |

### Modify: `packages/src/anytime_core/CMakeLists.txt`

Add test targets:

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_anytime_waitable test/test_anytime_waitable.cpp)
  target_include_directories(test_anytime_waitable PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_anytime_waitable rclcpp)

  ament_add_gtest(test_anytime_base test/test_anytime_base.cpp)
  target_include_directories(test_anytime_base PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_anytime_base rclcpp rclcpp_action)
endif()
```

---

## 3.2 Unit Tests for anytime_monte_carlo

### New File: `packages/src/anytime_monte_carlo/test/test_monte_carlo_management.cpp`

| Test | Description |
| ---- | ---- |
| `test_single_iteration_increments_counters` | One `compute_single_iteration()` increments `count_total_` and `loop_count_` |
| `test_should_finish_at_goal` | Set goal, iterate until `loop_count_ >= goal`, verify `should_finish()` returns true |
| `test_should_not_finish_before_goal` | Verify `should_finish()` returns false before reaching goal |
| `test_reset_clears_state` | Call `reset_domain_state()`, verify all counters zero |
| `test_populate_result_computes_pi` | After many iterations (e.g., 100000), verify result approximates pi (within tolerance) |
| `test_populate_feedback_returns_count` | Verify feedback `count` matches `count_total_` |
| `test_seeded_reproducibility` | Same seed produces same sequence of results |

### Modify: `packages/src/anytime_monte_carlo/CMakeLists.txt`

Add test targets (same pattern as anytime_core).

---

## Verification

```bash
# Build with tests
colcon build --packages-select anytime_core anytime_monte_carlo

# Run tests
colcon test --packages-select anytime_core anytime_monte_carlo

# View results
colcon test-result --verbose
```
