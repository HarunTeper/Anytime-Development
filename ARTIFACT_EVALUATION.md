# Artifact Evaluation Guide

**Paper:** Anytime ROS 2: Timely Task Completion in Non-Preemptive Robotic Systems
**Authors:** Harun Teper, TU Dortmund University
**Venue:** RTAS 2026
**Paper PDF:** [`main.pdf`](main.pdf) (included in repository root)

## Hardware Requirements

| Requirement | Minimum | Recommended |
| ----------- | ------- | ----------- |
| Architecture | x86_64 Linux | x86_64 Linux |
| RAM | 8 GB | 16 GB |
| Disk | 20 GB (Docker image + workspace) | 40 GB |
| Docker | Docker Engine 24+ | Docker Engine 24+ with Compose V2 |
| GPU (optional) | NVIDIA GPU with CUDA 12.5 support | NVIDIA GPU with 8+ GB VRAM |
| GPU toolkit | nvidia-container-toolkit | nvidia-container-toolkit |

**CPU-only mode** covers Figures 5a, 5b, 6, and Table I (Monte Carlo + Interference experiments).
**GPU required** only for Figures 7a, 7b (YOLO experiments).

## Quick Start

Build and enter the container:

```bash
# CPU-only (no GPU required)
docker compose build anytime-cpu
docker compose run --rm anytime-cpu bash

# OR with GPU support
docker compose build anytime-gpu
docker compose run --rm anytime-gpu bash
```

Inside the container, build the workspace and run the smoke test:

```bash
cd packages && colcon build --symlink-install && source install/setup.bash
cd ..
./scripts/smoke_test.sh
```

The smoke test validates the build, unit tests, config generation, a short Monte Carlo experiment, and LTTng trace collection. Expected duration: < 2 minutes.

## Reproducing Paper Figures

Each figure can be reproduced with a single command. The `--quick` flag uses fewer batch sizes and shorter runs to demonstrate the same trends faster.

| Paper Element | Command | Full Duration | Quick Mode | Hardware |
| ------------- | ------- | ------------- | ---------- | -------- |
| Figure 5a (segment count vs batch size) | `./scripts/reproduce_figure.sh 5a` | ~40 min | `--quick` ~5 min | CPU |
| Figure 5b (cancellation delay) | `./scripts/reproduce_figure.sh 5b` | ~40 min | `--quick` ~5 min | CPU |
| Figures 5a + 5b (both) | `./scripts/reproduce_figure.sh 5` | ~40 min | `--quick` ~5 min | CPU |
| Figures 6a + 6b + Table I (interference) | `./scripts/reproduce_figure.sh 6` | ~40 min | `--quick` ~3 min | CPU |
| Figure 7a (YOLO quality progression) | `./scripts/reproduce_figure.sh 7a` | ~30 min | N/A | GPU |
| Figure 7b (YOLO runtime comparison) | `./scripts/reproduce_figure.sh 7b` | ~1 hr | N/A | GPU |
| Figures 7a + 7b (both) | `./scripts/reproduce_figure.sh 7` | ~1.5 hrs | N/A | GPU |

### Quick Reproduction (CPU only, ~10 minutes)

Runs the smoke test, Monte Carlo experiments, and Interference experiments with reduced parameters:

```bash
./scripts/run_all.sh --quick --cpu-only
```

### Full Reproduction (~5+ hours with GPU)

Runs all experiments with full parameters, including the complete YOLO pipeline:

```bash
./scripts/run_all.sh --full
```

### Full Reproduction (CPU only, ~1.5 hours)

Runs Monte Carlo and Interference experiments with full parameters, skipping YOLO:

```bash
./scripts/run_all.sh --full --cpu-only
```

## Output Locations

After experiments complete, results are saved to the following locations:

### Figures

| Figure | Output File |
| ------ | ----------- |
| Figure 5a | `experiments/monte_carlo/results/plots/batch_size_vs_iterations.pdf` |
| Figure 5b | `experiments/monte_carlo/results/plots/cancellation_delay.pdf` |
| Figure 6a | `experiments/interference/results/plots/jitter_vs_batch_size.pdf` |
| Figure 6b | `experiments/interference/results/plots/compute_time_vs_batch_size.pdf` |
| Figure 7a | `experiments/yolo/results/quality_analysis/quality_ratio_progression.pdf` |
| Figure 7b | `experiments/yolo/results/runtime_analysis/throughput_comparison.png` |

### Data Exports

| Data | Output File |
| ---- | ----------- |
| Table I (interference metrics) | `experiments/interference/results/aggregated_results.csv` |
| Monte Carlo metrics (CSV) | `experiments/monte_carlo/results/aggregated_results.csv` |
| Monte Carlo metrics (JSON) | `experiments/monte_carlo/results/aggregated_results.json` |
| Monte Carlo individual runs | `experiments/monte_carlo/results/individual_runs.csv` |
| Interference individual runs | `experiments/interference/results/individual_runs.csv` |
| Interference metrics (JSON) | `experiments/interference/results/aggregated_results.json` |

### Additional Plots

Each experiment generates additional diagnostic plots beyond the paper figures:

- **Monte Carlo:** `experiments/monte_carlo/results/plots/` — throughput, latency breakdowns, timing distributions
- **Interference:** `experiments/interference/results/plots/` — timer period distributions, missed period rates
- **YOLO quality:** `experiments/yolo/results/quality_analysis/` — detection progression, layer timing, cancellation histograms
- **YOLO throughput:** `experiments/yolo/results/runtime_analysis/` — per-config stacked timing, cumulative runtime
- **YOLO cancellation:** `experiments/yolo/results/phase4_analysis/` — cancellation delay, layers processed, block size metrics

## YOLO Prerequisites (GPU only)

Before running YOLO experiments, download the model weights and test images:

```bash
# Download YOLO weights
cd packages/src/anytime_yolo
wget https://tu-dortmund.sciebo.de/s/W86QE9hUscsUPeM/download -O weights.zip
unzip -o weights.zip -d .
rm weights.zip
cd ../../..

# Download test images
mkdir -p packages/src/video_publisher/images
cd packages/src/video_publisher/images
wget https://tu-dortmund.sciebo.de/s/aA9MDhgN2lBmeZk/download -O images.zip
unzip -o images.zip -d .
rm images.zip
cd ../../../..
```

## Experiment Parameters

### Monte Carlo (Figures 5a, 5b)

| Parameter | Full | Quick |
| --------- | ---- | ----- |
| Batch sizes | 1024, 2048, 4096, 8192, 16384, 32768, 65536 | 1024, 16384, 65536 |
| Modes | reactive, proactive | reactive, proactive |
| Threading | single, multi | single, multi |
| Run duration | 10 seconds | 5 seconds |
| Total configs | 28 | 12 |

### Interference (Figure 6, Table I)

| Parameter | Full | Quick |
| --------- | ---- | ----- |
| Batch sizes | 1024, 2048, 4096, 8192, 16384, 32768, 65536 | 1024, 16384, 65536 |
| Modes | reactive, proactive | reactive, proactive |
| Threading | single | single |
| Timer period | 100 ms | 100 ms |
| Timer execution time | 10 ms | 10 ms |
| Run duration | 10 seconds | 5 seconds |
| Total configs | 14 | 6 |

### YOLO (Figures 7a, 7b)

| Parameter | Value |
| --------- | ----- |
| Network | YOLOv3 with 25 layers |
| Block sizes | 1, 8, 16, 25 |
| Sync modes | sync, async |
| Threading | single, multi |
| Mode | proactive |
| Baseline trials | 3 |
| Cancellation: cancel_after_layers | 25 |
| Cancellation: score_threshold | 0.7 |

## Environment Details

The Docker images provide a complete, self-contained environment:

- **OS:** Ubuntu 22.04 (Jammy)
- **ROS 2:** Humble Hawksbill (built from source)
- **DDS:** CycloneDDS (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`)
- **Build system:** colcon
- **Tracing:** LTTng (lttng-tools, liblttng-ust-dev, python3-babeltrace)
- **GPU image only:** CUDA 12.5.1, TensorRT 10.3.0
- **Python:** pandas, numpy (< 2.0), matplotlib

All tracing uses LTTng userspace tracepoints defined in `packages/src/anytime_tracing/`. Traces are collected automatically by the experiment scripts and parsed by the evaluation scripts using `babeltrace`.

## Troubleshooting

**Build fails with missing dependencies:**

```bash
cd packages
rm -rf build/ install/ log/
colcon build --symlink-install
```

**LTTng session already exists:**

```bash
lttng destroy
# Then re-run the experiment
```

**No GPU available but want to run CPU experiments:**

```bash
./scripts/run_all.sh --quick --cpu-only
# OR
./scripts/reproduce_figure.sh 5a --quick
```

**Permission denied on scripts:**

```bash
chmod +x scripts/*.sh experiments/monte_carlo/*.sh experiments/interference/*.sh experiments/yolo/*.sh
```

**Docker compose not found:**

Ensure Docker Compose V2 is installed. With modern Docker Engine, `docker compose` (without hyphen) should work. If using older Docker, install `docker-compose` separately.

**Traces directory not writable:**

LTTng requires the `tracing` group. Inside the container, the `vscode` user should already be a member. If not:

```bash
sudo usermod -aG tracing vscode
```

## Project Structure

```
Anytime-Development/
├── ARTIFACT_EVALUATION.md      # This guide
├── main.pdf                    # Accepted paper
├── docker-compose.yml          # Container management
├── packages/src/
│   ├── anytime_core/           # Base anytime computation framework
│   ├── anytime_interfaces/     # ROS 2 action type definitions
│   ├── anytime_monte_carlo/    # Monte Carlo pi estimation (CPU)
│   ├── anytime_yolo/           # Anytime YOLO object detection (GPU)
│   ├── anytime_tracing/        # LTTng tracepoint definitions
│   ├── experiments/            # Launch files and default configs
│   ├── interference/           # Timer interference test node
│   ├── video_publisher/        # Video frame publisher for YOLO
│   └── test_action/            # Test action for executor testing
├── experiments/
│   ├── monte_carlo/            # MC experiment scripts and evaluation
│   ├── interference/           # Interference experiment scripts and evaluation
│   └── yolo/                   # YOLO pipeline (9 steps)
└── scripts/
    ├── smoke_test.sh           # Quick validation (< 2 min)
    ├── run_all.sh              # End-to-end runner
    └── reproduce_figure.sh     # Per-figure reproduction
```
