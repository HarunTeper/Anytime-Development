# 08 — CI/CD Pipeline

**Day:** 3 (Thu Feb 26)
**Priority:** MEDIUM
**Estimated time:** 1-2 hours
**Depends on:** 05 (smoke test), 06 (Docker)

---

## Overview

Add a GitHub Actions workflow that builds the project and runs tests on every push/PR. CPU-only (no GPU in CI). Gives reviewers confidence that the codebase is in a working state.

---

## New File: `.github/workflows/ci.yml`

```yaml
name: CI

on:
  push:
    branches: [main, yolo3trace2fullexperiments]
  pull_request:
    branches: [main]

jobs:
  build-and-test:
    runs-on: ubuntu-22.04
    timeout-minutes: 60

    steps:
      - name: Checkout
        uses: actions/checkout@v4
        with:
          submodules: recursive

      - name: Build Docker image (CPU)
        run: docker compose build anytime-cpu

      - name: Build ROS2 workspace
        run: |
          docker compose run --rm anytime-cpu bash -c "
            cd /home/vscode/workspace/packages &&
            source /home/vscode/ros2_humble/install/setup.bash &&
            colcon build --symlink-install
          "

      - name: Run unit tests
        run: |
          docker compose run --rm anytime-cpu bash -c "
            cd /home/vscode/workspace/packages &&
            source /home/vscode/ros2_humble/install/setup.bash &&
            source install/setup.bash &&
            colcon test --packages-select anytime_core anytime_monte_carlo &&
            colcon test-result --verbose
          "

      - name: Run smoke test (Monte Carlo)
        run: |
          docker compose run --rm anytime-cpu bash -c "
            cd /home/vscode/workspace &&
            source /home/vscode/ros2_humble/install/setup.bash &&
            source packages/install/setup.bash &&
            CPU_ONLY=true ./scripts/smoke_test.sh
          "

      - name: Upload test results
        if: always()
        uses: actions/upload-artifact@v4
        with:
          name: test-results
          path: packages/build/*/test_results/
```

---

## Optional: Docker Image Caching

To speed up CI runs, cache the Docker image:

```yaml
      - name: Set up Docker Buildx
        uses: docker/setup-buildx-action@v3

      - name: Cache Docker layers
        uses: actions/cache@v4
        with:
          path: /tmp/.buildx-cache
          key: ${{ runner.os }}-buildx-${{ hashFiles('.devcontainer/linux-no-hardware/Dockerfile') }}
          restore-keys: |
            ${{ runner.os }}-buildx-
```

---

## Optional: Badge in README

After the workflow is set up, add a status badge to `README.md`:

```markdown
![CI](https://github.com/OWNER/Anytime-Development/actions/workflows/ci.yml/badge.svg)
```

---

## Verification

```bash
# Test locally with act (GitHub Actions local runner)
act -j build-and-test

# Or just verify the workflow YAML is valid
python3 -c "import yaml; yaml.safe_load(open('.github/workflows/ci.yml'))"
```
