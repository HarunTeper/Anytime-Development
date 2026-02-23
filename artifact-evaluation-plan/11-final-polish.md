# 11 — Final Polish

**Day:** 4 (Fri Feb 27)
**Priority:** MEDIUM
**Estimated time:** 2 hours
**Depends on:** All previous steps

---

## Overview

Final cleanup, formatting, and release preparation. This is the last step before submission.

---

## Tasks

### 11.1 Code Formatting

Run `clang-format` on all modified C++ files:

```bash
# Check which files were modified
git diff --name-only HEAD~10 -- '*.hpp' '*.cpp'

# Format them
clang-format -i packages/src/anytime_yolo/include/anytime_yolo/anytime_management.hpp
clang-format -i packages/src/anytime_yolo/include/anytime_yolo/yolo.hpp
clang-format -i packages/src/anytime_core/include/anytime_core/anytime_base.hpp
clang-format -i packages/src/anytime_core/include/anytime_core/anytime_waitable.hpp
# ... and all new test files
```

The project has a `.clang-format` file at the root — use it.

### 11.2 Script Permissions

Ensure all shell scripts are executable:

```bash
find . -name "*.sh" -exec chmod +x {} \;
```

### 11.3 Update README.md

**File:** `README.md`

Add/update:

- One-line artifact evaluation quick start: `pip install -e ./anytime_cli && anytime-eval run-all`
- Links to documentation: `docs/ARCHITECTURE.md`, `docs/EXPERIMENTS.md`, `docs/COMPONENTS.md`
- Link to `ARTIFACT_EVALUATION.md`
- Citation block:

```bibtex
@inproceedings{anytime-ros2-2026,
  title={Anytime ROS 2: Timely Task Completion in Non-Preemptive Robotic Systems},
  booktitle={RTAS 2026},
  year={2026}
}
```

- License section: "Licensed under Apache 2.0. See [LICENSE](LICENSE)."

### 11.4 Update .gitignore

**File:** `.gitignore`

Ensure these are covered:

```text
# Build artifacts
packages/build/
packages/install/
packages/log/

# Experiment output (large, regeneratable)
experiments/*/traces/
experiments/*/results/

# Python
__pycache__/
*.pyc
*.egg-info/
.pytest_cache/
dist/

# IDE
.vscode/settings.json
*.code-workspace

# OS
.DS_Store
Thumbs.db
```

### 11.5 Final Build and Test

```bash
# Clean build
cd packages && rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash

# Run all tests
colcon test
colcon test-result --verbose

# Run smoke test
./scripts/smoke_test.sh
```

### 11.6 Release Tag

```bash
git add -A
git commit -m "Prepare artifact evaluation submission"
git tag -a v1.0-artifact-evaluation -m "Artifact evaluation submission for RTAS 2026"
```

### 11.7 Verify Clean Checkout

Test that a fresh clone works:

```bash
cd /tmp
git clone /home/daes-enzo/Anytime-Development anytime-test
cd anytime-test
pip install -e ./anytime_cli
anytime-eval run-all --mode cpu --experiment smoke --dry-run
```

---

## Checklist

- [ ] All C++ files pass `clang-format` check
- [ ] All `.sh` scripts have execute permission
- [ ] README.md has quick start, docs links, citation, license
- [ ] .gitignore covers all generated artifacts
- [ ] `colcon build` succeeds from clean state
- [ ] `colcon test` all tests pass
- [ ] Smoke test passes
- [ ] `anytime-eval --help` works after `pip install`
- [ ] Git tag created
- [ ] Fresh clone + dry-run works
