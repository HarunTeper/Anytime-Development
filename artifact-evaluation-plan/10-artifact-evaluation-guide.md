# 10 — Artifact Evaluation Guide

**Day:** 4 (Fri Feb 27)
**Priority:** HIGH
**Estimated time:** 1 hour
**Depends on:** 07 (Python CLI)

---

## Overview

Create a reviewer-facing document that maps paper claims to experiments and provides clear instructions for validation. This is the first thing AE reviewers will read.

---

## New File: `ARTIFACT_EVALUATION.md` (repository root)

### Contents

```markdown
# Artifact Evaluation Guide

## Paper
"Anytime ROS 2: Timely Task Completion in Non-Preemptive Robotic Systems" — RTAS 2026

## Claims and Experiments

| # | Paper Claim | Section | Experiment | Expected Result |
|---|-------------|---------|------------|-----------------|
| 1 | Anytime algorithms reduce task interference | VIII | Monte Carlo + Interference | Smaller batch sizes → lower timer jitter |
| 2 | Reactive and proactive modes trade off latency vs memory | VIII | Monte Carlo | Proactive: lower cancel latency. Reactive: less memory |
| 3 | Cooperative notification outperforms passive polling | VI-B | YOLO (sync vs async) | Async throughput ≥ sync throughput |
| 4 | Quality-based cancellation achieves target quality with fewer layers | IX-C | YOLO Phase 4 | Score threshold met at layer 8-16 (not 25) |
| 5 | Cancellation delay scales with batch size | IX-B | Monte Carlo + YOLO Phase 4 | Larger batch → higher cancel delay |
| 6 | Multi-threaded executor reduces interference | IX-A | Interference | Multi-threaded: fewer missed timer periods |

## Hardware Requirements

| Mode | Requirements | Experiments Available |
|------|-------------|---------------------|
| CPU  | x86_64 Linux, 16GB RAM, 20GB disk, Docker | Monte Carlo, Interference |
| GPU  | Above + NVIDIA GPU (CUDA 12.5), 50GB disk | All (Monte Carlo, Interference, YOLO) |

## Quick Start

### Prerequisites
- Docker Engine installed
- (For GPU) NVIDIA Container Toolkit installed

### Installation
```bash
pip install -e ./anytime_cli
```

### Quick Validation (~5 min)
```bash
anytime-eval run-all --mode cpu --experiment smoke
```

This builds the container, runs unit tests, and executes a short Monte Carlo experiment.

### CPU-Only Reproduction (~2 hours)
```bash
anytime-eval run-all --mode cpu --experiment monte_carlo
anytime-eval run-all --mode cpu --experiment interference
```

Validates Claims 1, 2, 5, 6.

### Full Reproduction (~5 hours, requires GPU)
```bash
anytime-eval run-all --mode gpu --experiment all
```

Validates all claims including YOLO experiments (Claims 3, 4).

## Output

Results are generated in:
- `experiments/monte_carlo/results/` — CSV, JSON, plots
- `experiments/interference/results/` — CSV, JSON, plots
- `experiments/yolo/results/` — CSV, JSON, plots (quality, throughput, cancellation)

### Key Output Files

| File | Description | Validates |
|------|-------------|-----------|
| `monte_carlo/results/plots/throughput_vs_batch_size.png` | Batch size scaling | Claim 5 |
| `monte_carlo/results/plots/cancellation_delay.png` | Cancel latency by mode | Claim 2 |
| `interference/results/plots/timer_period_vs_batch_size.png` | Interference severity | Claims 1, 6 |
| `yolo/results/quality_analysis/quality_progression.png` | Layer-wise detection quality | Claim 4 |
| `yolo/results/runtime_analysis/throughput_comparison.png` | Sync vs async throughput | Claim 3 |
| `yolo/results/phase4_analysis/cancellation_delay.png` | YOLO cancel performance | Claims 4, 5 |

## Troubleshooting

### Docker build fails
- Ensure Docker Engine (not Docker Desktop) is installed
- Check available disk space: `df -h`

### GPU not detected
- Verify NVIDIA driver: `nvidia-smi`
- Verify container toolkit: `docker run --rm --gpus all nvidia/cuda:12.5.0-base-ubuntu22.04 nvidia-smi`

### Experiment produces no traces
- Verify LTTng is working: `lttng list -u` (inside container)
- Check tracing group membership: `groups` should include `tracing`

### Results differ from paper
- Small numerical differences are expected due to stochastic algorithms
- Monte Carlo: pi estimates converge with more iterations
- Timing results depend on hardware — trends should match, absolute values may differ
```

---

## Verification

- Review with a fresh perspective: can a reviewer follow this without prior knowledge?
- Test each "Quick Start" command end-to-end
- Verify all output file paths exist after running experiments
- Check troubleshooting steps against known issues
