# 12 — Additional Items

**Days:** Spread across Days 1-3
**Priority:** Mixed (see per-item)

---

## Overview

Additional value-add items beyond the core 6 requirements. These improve the artifact evaluation experience for reviewers and increase the chance of earning badges.

---

## 12.1 Pre-built Docker Images on GHCR

**Priority:** HIGH
**Day:** 3 (after Docker restructuring)
**Estimated time:** 1-2 hours

Push pre-built images to GitHub Container Registry so reviewers don't need to build from source (~1 hour saved).

### Steps

1. Build images locally:

```bash
docker compose build anytime-cpu
docker compose build anytime-gpu
```

2. Tag and push:

```bash
docker tag anytime-cpu ghcr.io/OWNER/anytime-ros2:cpu-latest
docker tag anytime-gpu ghcr.io/OWNER/anytime-ros2:gpu-latest

docker push ghcr.io/OWNER/anytime-ros2:cpu-latest
docker push ghcr.io/OWNER/anytime-ros2:gpu-latest
```

3. Update `docker-compose.yml` to use pre-built images by default:

```yaml
services:
  anytime-cpu:
    image: ghcr.io/OWNER/anytime-ros2:cpu-latest
    build:
      context: .
      dockerfile: .devcontainer/linux-no-hardware/Dockerfile
```

4. Update Python CLI to pull image first, build only if pull fails.

### Verification

```bash
# Pull from GHCR
docker pull ghcr.io/OWNER/anytime-ros2:cpu-latest

# Run smoke test with pulled image
docker run --rm ghcr.io/OWNER/anytime-ros2:cpu-latest \
  bash -c "cd /workspace && colcon build && colcon test"
```

---

## 12.2 Requirements File with Pinned Versions

**Priority:** HIGH
**Day:** 1 (during refactoring)
**Estimated time:** 15 minutes

### New File: `requirements.txt`

```text
# Analysis dependencies
pandas==2.0.3
numpy==1.24.4
matplotlib==3.7.5
pyyaml==6.0.1
babeltrace2>=0.6.0

# CLI tool
click>=8.0

# YOLO (optional, for visualization)
opencv-python-headless==4.8.1.78
```

### Integration

- Reference in Dockerfile: `COPY requirements.txt . && pip install -r requirements.txt`
- Reference in `pyproject.toml`: add as dependencies
- Reference in `ARTIFACT_EVALUATION.md`: mention for manual install

---

## 12.3 LICENSE File

**Priority:** HIGH (required for artifact badges)
**Day:** 1 (during refactoring)
**Estimated time:** 5 minutes

### New File: `LICENSE`

Apache License 2.0 full text. Available at: https://www.apache.org/licenses/LICENSE-2.0.txt

Ensure all `package.xml` files reference `Apache-2.0` consistently (step 02).

---

## 12.4 Reproducible Seeds for Monte Carlo

**Priority:** MEDIUM
**Day:** 1 (during refactoring)
**Estimated time:** 30 minutes

### Changes

**File:** `packages/src/anytime_monte_carlo/include/anytime_monte_carlo/anytime_management.hpp`

Add `random_seed` parameter:

```cpp
// In constructor or on_activate():
this->node_->declare_parameter("random_seed", 42);
int seed = this->node_->get_parameter("random_seed").as_int();
srand(seed);
RCLCPP_INFO(this->node_->get_logger(), "Random seed: %d", seed);
```

**File:** `packages/src/experiments/config/monte_carlo/default_server.yaml`

```yaml
anytime_server:
  ros__parameters:
    random_seed: 42
```

**Also update:** All generated experiment configs in `experiments/monte_carlo/generate_configs.py` to include the seed parameter.

### Verification

```bash
# Run twice with same seed, compare pi estimates
# Results should be identical for same seed + same batch_size + same iterations
```

---

## 12.5 Performance Regression Baseline

**Priority:** LOW
**Day:** 3 (optional addition to CLI)
**Estimated time:** 2 hours

### Concept

Store baseline timing results (from a known-good run) and compare new experiment results against them.

### New File: `baselines/monte_carlo_baseline.json`

```json
{
  "hardware": "Intel i7-12700K, 32GB RAM",
  "date": "2026-02-26",
  "metrics": {
    "batch_1024_reactive_single": {
      "throughput_mean": 125000,
      "throughput_tolerance_pct": 20
    }
  }
}
```

### New CLI Subcommand: `anytime-eval benchmark`

```text
Usage: anytime-eval benchmark [OPTIONS]

Options:
  --baseline PATH    Path to baseline JSON (default: baselines/monte_carlo_baseline.json)
  --update           Update baseline with current results
```

Compares current experiment results against baseline, reports deviations beyond tolerance.

---

## 12.6 Artifact Evaluation Badges

**Priority:** LOW
**Day:** 4 (during final polish)
**Estimated time:** 15 minutes

Add badge images to README.md once AE results are in:

```markdown
![Artifacts Available](https://img.shields.io/badge/Artifacts-Available-green)
![Artifacts Evaluated](https://img.shields.io/badge/Artifacts-Evaluated-green)
![Results Reproduced](https://img.shields.io/badge/Results-Reproduced-green)
```

These are aspirational — add placeholders now, update with real badges after AE review.

---

## Priority Summary

| Item | Priority | Day | Time |
| ---- | ---- | ---- | ---- |
| 12.2 requirements.txt | HIGH | 1 | 15 min |
| 12.3 LICENSE | HIGH | 1 | 5 min |
| 12.4 Reproducible seeds | MEDIUM | 1 | 30 min |
| 12.1 Pre-built Docker images | HIGH | 3 | 1-2 hrs |
| 12.5 Performance baselines | LOW | 3 | 2 hrs |
| 12.6 Badges | LOW | 4 | 15 min |
