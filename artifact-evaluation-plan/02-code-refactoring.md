# 02 — Code Refactoring Overview

**Day:** 1-2 (Tue-Wed Feb 24-25)
**Priority:** HIGH
**Estimated time:** 4-5 hours

---

## Scope

Fix all code quality issues found during deep code review. Organized by severity — correctness bugs first, then code quality, then experiment scripts.

---

## Part A: C++ Correctness Bugs (Must Fix)

### C1. Null pointer dereference in AnytimeClientBase

**File:** `packages/src/anytime_core/include/anytime_core/anytime_client_base.hpp`

**Lines 68-69** (`log_result()`): `goal_handle_->get_goal_id()` dereferenced without null check.
**Line 129** (CANCELED branch in `result_callback()`): Same issue.

`goal_handle_` is only set in `goal_response_callback()` at line 208. If result arrives in an unexpected order, this is UB.

**Fix:** Add null guard:

```cpp
// In log_result() — line 67-70
virtual void log_result(const typename AnytimeGoalHandle::WrappedResult & result)
{
  (void)result;
  if (goal_handle_) {
    RCLCPP_DEBUG(
      this->get_logger(), "[Goal ID: %s] Result received",
      rclcpp_action::to_string(goal_handle_->get_goal_id()).c_str());
  }
}

// In result_callback() CANCELED case — line 127-131
case rclcpp_action::ResultCode::CANCELED:
  if (goal_handle_) {
    RCLCPP_DEBUG(
      this->get_logger(), "[Goal ID: %s] Goal was canceled",
      rclcpp_action::to_string(goal_handle_->get_goal_id()).c_str());
  }
  post_processing(result);
  break;
```

---

### C2. Unsafe bounds access in YOLO populate_result()

**File:** `packages/src/anytime_yolo/include/anytime_yolo/anytime_management.hpp`

**Lines 120-135:** The loop accesses `yolo_result[i+4]` at line 121 and 131 before the `i + 5 >= size()` safety check at line 135.

**Fix:** Move bounds check to top of loop body:

```cpp
for (size_t i = 0; i < yolo_result.size(); i += 6) {
  if (i + 5 >= yolo_result.size()) break;  // Move to TOP

  // Now safe to access [i] through [i+5]
  if (yolo_result[i + 4] == 0.0) {
    continue;
  }
  // ... rest of detection parsing
}
```

Also apply this to the counting loop at lines 120-124:

```cpp
for (size_t i = 0; i + 5 < yolo_result.size(); i += 6) {
  if (yolo_result[i + 4] > 0.0) {
    detection_count++;
  }
}
```

---

### C3. cudaMemset after potentially failed cudaMalloc

**File:** `packages/src/anytime_yolo/include/anytime_yolo/yolo.hpp`

**Lines 171-177:** `cudaMemset` on line 172 runs before the error check on line 174.

**Fix:**

```cpp
bool allocate(size_t size)
{
  if (size == this->size) return true;

  free();
  this->size = size;
  cudaError_t status = cudaMalloc(&device_ptr, size);
  if (status != cudaSuccess) {
    std::cerr << "Error allocating CUDA memory: " << cudaGetErrorString(status) << std::endl;
    device_ptr = nullptr;
    this->size = 0;
    return false;
  }
  cudaMemset(device_ptr, 0, size);  // Only after success
  return true;
}
```

---

### C4. Variable shadowing in inferStep()

**File:** `packages/src/anytime_yolo/include/anytime_yolo/yolo.hpp`

**Line 1066:** `auto stream = this->stream;` shadows the class member.

**Fix:** Remove the redundant local variable:

```cpp
// Before:
auto stream = this->stream;
cudaLaunchHostFunc(stream, callback, userData);

// After:
if (callback != nullptr) {
  cudaLaunchHostFunc(this->stream, callback, userData);
}
```

---

### C5. cudaLaunchHostFunc always called (also in 01-gpu-sync-fix.md)

**File:** `packages/src/anytime_yolo/include/anytime_yolo/yolo.hpp`

**Lines 1066-1067:** Called unconditionally — even when `callback` is nullptr (sync mode) and even in the COMPLETED stage.

**Fix:** Guard with null check (combined with C4 fix above). This is also addressed in step 01 but listed here for completeness.

---

## Part B: C++ Code Quality (Should Fix)

### C6. Inefficient Monte Carlo math

**File:** `packages/src/anytime_monte_carlo/include/anytime_monte_carlo/anytime_management.hpp`

**Line 41:** `sqrt(pow(x, 2) + pow(y, 2)) <= 1` — 3 function calls for a simple distance check.

**Fix:**

```cpp
// Before:
if (sqrt(pow(x, 2) + pow(y, 2)) <= 1) {

// After (mathematically equivalent, ~10x faster):
if (x * x + y * y <= 1.0) {
```

---

### C7. Unused member variable

**File:** `packages/src/anytime_monte_carlo/include/anytime_monte_carlo/anytime_management.hpp`

**Line 94:** `count_outside_` is declared, reset in `reset_domain_state()` (line 81), but never incremented or read.

**Fix:** Remove `count_outside_` declaration (line 94) and its reset (line 81).

---

### C8. Duplicate MAX_NETWORK_LAYERS constant

**File:** `packages/src/anytime_yolo/include/anytime_yolo/anytime_management.hpp`

**Line 31:** `static constexpr int MAX_NETWORK_LAYERS = 25;` (class level)
**Line 80:** `constexpr int MAX_NETWORK_LAYERS = 25;` (local redefinition in `get_batch_iterations()`)

**Fix:** Remove line 80, use the class-level constant:

```cpp
int get_batch_iterations() const override
{
  int layers_left = MAX_NETWORK_LAYERS - processed_layers_;  // Use class constant
  return std::min(this->batch_size_, layers_left);
}
```

---

### C9. CMakeLists comment mismatch

**File:** `packages/src/anytime_yolo/CMakeLists.txt`

**Line 4:** Comment says "Default to C++14" but code sets C++20.

**Fix:** Change comment to `# Default to C++20`.

---

### C10. notify_cancel() is a no-op

**File:** `packages/src/anytime_core/include/anytime_core/anytime_base.hpp`

**Lines 253-262:** `notify_waitable()` call is commented out. The function does nothing but log.

**Fix:** Remove the dead `notify_cancel()` method entirely. If cancellation notification is needed later, implement it properly.

---

## Part C: Original Refactoring Items

### 2.1 Package Metadata Cleanup

**Files:** All `package.xml` files in custom packages

| Package | File | Fix |
| ---- | ---- | ---- |
| anytime_core | `packages/src/anytime_core/package.xml` | Replace placeholder maintainer/email |
| anytime_yolo | `packages/src/anytime_yolo/package.xml` | Replace `<license>TODO</license>` with `Apache-2.0` |
| anytime_monte_carlo | `packages/src/anytime_monte_carlo/package.xml` | Set real maintainer, license |
| anytime_interfaces | `packages/src/anytime_interfaces/package.xml` | Set real maintainer, license |
| anytime_tracing | `packages/src/anytime_tracing/package.xml` | Set real maintainer, license |
| interference | `packages/src/interference/package.xml` | Set real maintainer, license |
| experiments | `packages/src/experiments/package.xml` | Set real maintainer, license |
| video_publisher | `packages/src/video_publisher/package.xml` | Set real maintainer, license |

### 2.2 Strict Aliasing Violation

**File:** `packages/src/anytime_yolo/include/anytime_yolo/anytime_management.hpp:254-261`

`get_iteration_callback()` uses `reinterpret_cast<void*>(&forward_finished_callback)` — undefined behavior.

**Fix:** Remove `get_iteration_callback()` from both YOLO management and `anytime_base.hpp:34`. After the GPU sync fix (step 01), the callback is passed directly in `compute_single_iteration()`.

### 2.3 Reproducible Random Seeds

**File:** `packages/src/anytime_monte_carlo/include/anytime_monte_carlo/anytime_management.hpp`

`rand()/RAND_MAX` used without seeding.

**Fix:**

- Add `random_seed` ROS2 parameter (default: `42`)
- Call `srand(seed)` during initialization
- Update `packages/src/experiments/config/monte_carlo/default_server.yaml`

### 2.4 Add LICENSE File

**New file:** `LICENSE` — Apache 2.0 text at repository root.

### 2.5 Add requirements.txt

**New file:** `requirements.txt`

```text
pandas==2.0.3
numpy==1.24.4
matplotlib==3.7.5
pyyaml==6.0.1
babeltrace2>=0.6.0
click>=8.0
opencv-python-headless==4.8.1.78
```

---

## Part D: Experiment Script Fixes

### E1. Configuration comment/code mismatch

**File:** `experiments/yolo/5_generate_configs.py:40-47`

Comments say "Cancel after 16 layers" / "Threshold of 0.8" but code sets `cancel_after_layers: 25` and `score_threshold: 0.7`.

**Fix:** Sync comments with actual values.

### E2. Missing signal trap handlers

**Files:** ALL experiment `.sh` scripts (9 files listed in section 2.4 of original plan)

**Fix:** Add to every runner script:

```bash
cleanup() {
  echo "Cleaning up..."
  kill ${LAUNCH_PID} 2>/dev/null || true
  kill ${VIDEO_PUB_PID} 2>/dev/null || true
  lttng stop "${SESSION_NAME}" 2>/dev/null || true
  lttng destroy "${SESSION_NAME}" 2>/dev/null || true
}
trap cleanup SIGINT SIGTERM EXIT
```

### E3. Aggressive pkill patterns

**Files:** All runner scripts

`pkill -9 -f 'ros2'` matches any process with "ros2" in args.

**Fix:** Track PIDs explicitly and kill those, or use more specific patterns like `pkill -9 -f "anytime_server"`.

### E4. Duplicated trace parsing code

**Files:**

- `experiments/monte_carlo/evaluate_monte_carlo.py` (`parse_trace_directory()`)
- `experiments/interference/evaluate_interference.py` (`parse_trace_directory()`)
- `experiments/yolo/7_analyze_cancellation.py` (`parse_trace_directory()`)

**Fix:** Extract to `experiments/lib/trace_parser.py`:

```python
# experiments/lib/__init__.py
# experiments/lib/trace_parser.py

def find_babeltrace():
    """Find babeltrace2 or babeltrace command."""
    ...

def parse_trace_directory(trace_dir, event_filter=None):
    """Parse LTTng trace directory and return structured events."""
    ...
```

### E5. Duplicated plot generation code

**Files:**

- `experiments/monte_carlo/evaluate_monte_carlo.py` (~500 lines)
- `experiments/interference/evaluate_interference.py` (~500 lines)

**Fix:** Extract to `experiments/lib/plotting.py`:

```python
def create_grouped_bar_plot(df, metric, title, output_path, group_by='mode'):
    ...

def setup_matplotlib_defaults():
    ...
```

### E6. Deprecated matplotlib style

**Files:** All Python evaluation scripts

`plt.style.use('seaborn-v0_8-darkgrid')` — deprecated.

**Fix:**

```python
try:
    plt.style.use('seaborn-v0_8-darkgrid')
except OSError:
    plt.style.use('default')
```

### E7. Python scripts hardcode workspace paths

**Files:** All `.py` files in `experiments/` (14+ files)

**Fix:** Use `Path(__file__).parent` for relative paths:

```python
SCRIPT_DIR = Path(__file__).parent
WORKSPACE_DIR = Path(os.environ.get('WORKSPACE_DIR', SCRIPT_DIR.parent.parent))
```

### E8. Missing LTTng availability check

**Files:** All `.sh` experiment scripts

**Fix:** Add at script start:

```bash
command -v lttng >/dev/null 2>&1 || { echo "Error: LTTng not installed"; exit 1; }
```

### E9. Unused Python imports

**Files:**

- `experiments/monte_carlo/evaluate_monte_carlo.py` — `ProcessPoolExecutor`, `multiprocessing` imported but unused
- `experiments/interference/evaluate_interference.py` — `re` potentially unused

**Fix:** Remove unused imports.

---

## Verification

```bash
# C++ changes compile
colcon build --symlink-install

# No new compiler warnings
colcon build --cmake-args -DCMAKE_CXX_FLAGS="-Wall -Wextra"

# Experiment scripts still work with default paths
cd experiments/monte_carlo && ./test_single_config.sh

# Python imports clean
python3 -c "from experiments.lib.trace_parser import parse_trace_directory"

# Random seed reproducibility
# Run Monte Carlo twice with same seed, compare pi estimates
```

---

## Execution Order

1. **C1-C5** (correctness bugs) — first, before any testing
2. **2.2** (strict aliasing) — alongside step 01 GPU sync fix
3. **C6-C10** (code quality) — quick fixes
4. **2.1, 2.4, 2.5** (metadata, LICENSE, requirements.txt) — quick
5. **E1-E3** (experiment correctness) — before running experiments
6. **2.3** (random seeds) — before running Monte Carlo experiments
7. **E4-E9** (shared libraries, cleanup) — can defer to Day 4 if needed
