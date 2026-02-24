# Step 8: Code Quality + Metadata

**Status:** Completed
**Day:** 3 (Thu Feb 26)
**Priority:** MEDIUM
**Time:** ~1h
**Dependencies:** None

## Why

Clean code and proper metadata matter for Criterion 3 (Quality).

**AE Impact:** Criterion 3 - "purpose of almost all files documented, clear names."

## Quick C++ Fixes

| ID | File | Fix |
| -- | ---- | --- |
| C6 | `anytime_monte_carlo/.../anytime_management.hpp:41` | `sqrt(pow(x,2)+pow(y,2))` -> `x*x + y*y <= 1.0` |
| C7 | `anytime_monte_carlo/.../anytime_management.hpp:94` | Remove unused `count_outside_` |
| C8 | `anytime_yolo/.../anytime_management.hpp:80` | Remove duplicate `MAX_NETWORK_LAYERS` |
| C9 | `anytime_yolo/CMakeLists.txt:4` | Fix comment "C++14" -> "C++20" |
| C10 | `anytime_core/.../anytime_base.hpp:253-262` | Remove dead `notify_cancel()` |

## Package Metadata

All `package.xml` files (8 packages):

- Set real maintainer name/email (replace placeholders)
- Set license to `Apache-2.0` consistently

## Reproducible Seeds

- `anytime_monte_carlo/.../anytime_management.hpp` - Add `random_seed` ROS2 parameter (default: 42), call `srand(seed)`
- Update `experiments/config/monte_carlo/default_server.yaml` with `random_seed: 42`
- Update `experiments/monte_carlo/generate_configs.py` to include seed in generated configs

## Experiment Script Fixes

- Add signal trap handlers (`trap cleanup SIGINT SIGTERM EXIT`) to all runner scripts
- Replace aggressive `pkill -9 -f 'ros2'` with specific process targeting
- Fix comment/code mismatch in `experiments/yolo/5_generate_configs.py:40-47`
- Add LTTng availability check at start of runner scripts
- Remove unused Python imports
