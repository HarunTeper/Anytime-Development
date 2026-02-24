# Artifact Evaluation Guide

**Paper:** Anytime ROS 2: Timely Task Completion in Non-Preemptive Robotic Systems
**Authors:** Harun Teper, TU Dortmund University
**Venue:** RTAS 2026
**Paper PDF:** [`main.pdf`](main.pdf) (included in repository root)

## Overview

This artifact can be evaluated through three paths depending on available hardware. **Path A (Jetson via SSH) is recommended** — the board is pre-configured and ready to use.

| Path | Hardware | Figures Covered | Setup Time |
| ---- | -------- | --------------- | ---------- |
| **A: Jetson via SSH (Recommended)** | Provided by authors | All (5a, 5b, 6, 7a, 7b, Table I) | ~5 min |
| **B: Local with NVIDIA GPU** | NVIDIA desktop GPU + Docker | All (5a, 5b, 6, 7a, 7b, Table I) | ~30–60 min (build) |
| **C: Local CPU only** | Any x86_64 Linux + Docker | CPU only (5a, 5b, 6, Table I) | ~20–40 min (build) |

---

## Path A: NVIDIA Jetson Board via SSH (Recommended)

An NVIDIA Jetson Orin NX board is provided at TU Dortmund University with everything pre-configured: Docker image built, workspace compiled, YOLO weights and test images downloaded.

> **Credentials:** SSH credentials are provided in a **separate confidential document** submitted alongside this artifact. They are intentionally not included in this repository. If you did not receive credentials, please contact the authors.

> **Shared access:** Only one Jetson board is available. If multiple reviewers are evaluating concurrently, please coordinate to avoid running experiments simultaneously on the same board, as this would affect timing results.

### Connect to the Jetson board

```bash
ssh -p <JETSON_PORT> <JETSON_USERNAME>@<JETSON_IP>
# Password: <JETSON_PASSWORD>
```

Replace the `<...>` placeholders with the values from the confidential credentials document.

### Enter the Docker container

```bash
cd ~/Anytime-Development
docker compose run --rm anytime-jetson bash
```

### Build the workspace and run experiments

Inside the container:

```bash
cd packages && colcon build --symlink-install && source install/setup.bash
cd ..

# Run smoke test (< 2 minutes)
./scripts/smoke_test.sh

# Quick CPU experiments (~10 minutes)
./scripts/run_all.sh --quick --cpu-only

# Full reproduction including YOLO (~5+ hours)
./scripts/run_all.sh --full

# Run only GPU experiments (~3 hours)
./scripts/run_all.sh --full --gpu-only

# Run specific experiment groups
./scripts/run_all.sh --full --monte-carlo
./scripts/run_all.sh --full --interference
./scripts/run_all.sh --full --yolo

# Or reproduce individual figures (see table below)
./scripts/reproduce_figure.sh 5a --quick
```

YOLO weights and test images are **pre-downloaded** on the board. No additional setup is needed for GPU experiments.

If you wish to re-download them (e.g., to verify integrity), see [YOLO Prerequisites](#yolo-prerequisites-gpu-only--paths-a--b) below.

### Copy results to your local machine

Copy just the paper figures (recommended):

```bash
scp -P <JETSON_PORT> -r <JETSON_USERNAME>@<JETSON_IP>:~/Anytime-Development/paper_figures/ ./paper_figures/
```

Or copy all experiment results (includes additional diagnostic plots and raw data):

```bash
scp -P <JETSON_PORT> -r <JETSON_USERNAME>@<JETSON_IP>:~/Anytime-Development/experiments/*/results/ ./results/
```

---

## Path B: Local Machine with NVIDIA GPU

### Hardware requirements

| Requirement | Minimum | Recommended |
| ----------- | ------- | ----------- |
| Architecture | x86_64 Linux | x86_64 Linux |
| RAM | 8 GB | 16 GB |
| Disk | 20 GB | 40 GB |
| Docker | Docker Engine 24+ with Compose V2 | Docker Engine 24+ with Compose V2 |
| GPU | NVIDIA GPU with CUDA 12.5 support | NVIDIA GPU with 8+ GB VRAM |
| GPU driver | NVIDIA driver 535+ | NVIDIA driver 535+ |
| GPU toolkit | [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) | nvidia-container-toolkit |

### Build and enter the container

```bash
git clone <repository-url>
cd Anytime-Development

docker compose build anytime-gpu
docker compose run --rm anytime-gpu bash
```

### Build the workspace and run experiments

Inside the container:

```bash
cd packages && colcon build --symlink-install && source install/setup.bash
cd ..

# Run smoke test (< 2 minutes)
./scripts/smoke_test.sh
```

Before running YOLO experiments, download the model weights and test images — see [YOLO Prerequisites](#yolo-prerequisites-gpu-only--paths-a--b) below.

Then run experiments:

```bash
# Quick CPU experiments (~10 minutes)
./scripts/run_all.sh --quick --cpu-only

# Full reproduction including YOLO (~5+ hours)
./scripts/run_all.sh --full

# Run only GPU experiments (~3 hours)
./scripts/run_all.sh --full --gpu-only

# Run specific experiment groups
./scripts/run_all.sh --full --monte-carlo
./scripts/run_all.sh --full --interference
./scripts/run_all.sh --full --yolo

# Or reproduce individual figures (see table below)
./scripts/reproduce_figure.sh 5a --quick
```

---

## Path C: Local Machine — CPU Only

This path reproduces Figures 5a, 5b, 6, and Table I. Figures 7a and 7b (YOLO) require a GPU and are **not available** on this path.

### Hardware requirements

| Requirement | Minimum | Recommended |
| ----------- | ------- | ----------- |
| Architecture | x86_64 Linux | x86_64 Linux |
| RAM | 8 GB | 16 GB |
| Disk | 10 GB | 20 GB |
| Docker | Docker Engine 24+ with Compose V2 | Docker Engine 24+ with Compose V2 |

### Build and enter the container

```bash
git clone <repository-url>
cd Anytime-Development

docker compose build anytime-cpu
docker compose run --rm anytime-cpu bash
```

### Build the workspace and run experiments

Inside the container:

```bash
cd packages && colcon build --symlink-install && source install/setup.bash
cd ..

# Run smoke test (< 2 minutes)
./scripts/smoke_test.sh

# Quick CPU experiments (~10 minutes)
./scripts/run_all.sh --quick --cpu-only

# Full CPU experiments (~1.5 hours)
./scripts/run_all.sh --full --cpu-only

# Or reproduce individual figures (see table below)
./scripts/reproduce_figure.sh 5a --quick
```

---

## Reproducing Paper Figures

Each figure can be reproduced with a single command. The `--quick` flag uses fewer batch sizes and shorter runs to demonstrate the same trends faster.

| Paper Element | Command | Full Duration | Quick Mode | Platforms |
| ------------- | ------- | ------------- | ---------- | --------- |
| Figure 5a (segment count vs batch size) | `./scripts/reproduce_figure.sh 5a` | ~40 min | `--quick` ~5 min | A, B, C |
| Figure 5b (cancellation delay) | `./scripts/reproduce_figure.sh 5b` | ~40 min | `--quick` ~5 min | A, B, C |
| Figures 5a + 5b (both) | `./scripts/reproduce_figure.sh 5` | ~40 min | `--quick` ~5 min | A, B, C |
| Figures 6a + 6b + Table I (interference) | `./scripts/reproduce_figure.sh 6` | ~40 min | `--quick` ~3 min | A, B, C |
| Figure 7a (YOLO quality progression) | `./scripts/reproduce_figure.sh 7a` | ~30 min | N/A | A, B |
| Figure 7b (YOLO runtime comparison) | `./scripts/reproduce_figure.sh 7b` | ~1 hr | N/A | A, B |
| Figures 7a + 7b (both) | `./scripts/reproduce_figure.sh 7` | ~1.5 hrs | N/A | A, B |

## Output Locations

After experiments complete, all paper figures and Table I are automatically collected into a single directory for easy access:

```
paper_figures/
├── figure_5a_batch_size_vs_iterations.pdf
├── figure_5b_cancellation_delay.pdf
├── figure_6a_jitter_vs_batch_size.pdf
├── figure_6b_compute_time_vs_batch_size.pdf
├── figure_7a_quality_ratio_progression.pdf
├── figure_7b_throughput_comparison.png
└── table_1_interference_metrics.csv
```

You can also re-collect figures at any time by running:

```bash
./scripts/collect_figures.sh
```

### Detailed output locations

The full experiment results (including additional diagnostic plots and raw data) remain in their respective experiment directories:

| Figure / Data | Output File |
| ------------- | ----------- |
| Figure 5a | `experiments/monte_carlo/results/plots/batch_size_vs_iterations.pdf` |
| Figure 5b | `experiments/monte_carlo/results/plots/cancellation_delay.pdf` |
| Figure 6a | `experiments/interference/results/plots/jitter_vs_batch_size.pdf` |
| Figure 6b | `experiments/interference/results/plots/compute_time_vs_batch_size.pdf` |
| Figure 7a | `experiments/yolo/results/quality_analysis/quality_ratio_progression.pdf` |
| Figure 7b | `experiments/yolo/results/runtime_analysis/throughput_comparison.png` |
| Table I (CSV) | `experiments/interference/results/aggregated_results.csv` |

### Additional diagnostic plots

Each experiment generates additional plots beyond the paper figures:

- **Monte Carlo:** `experiments/monte_carlo/results/plots/` — throughput, latency breakdowns, timing distributions
- **Interference:** `experiments/interference/results/plots/` — timer period distributions, missed period rates
- **YOLO quality:** `experiments/yolo/results/quality_analysis/` — detection progression, layer timing, cancellation histograms
- **YOLO throughput:** `experiments/yolo/results/runtime_analysis/` — per-config stacked timing, cumulative runtime
- **YOLO cancellation:** `experiments/yolo/results/phase4_analysis/` — cancellation delay, layers processed, block size metrics

## YOLO Prerequisites (GPU only — Paths A & B)

> **Path A note:** The Jetson board has weights and images **pre-downloaded**. You can skip this section unless you want to re-download them.

Before running YOLO experiments on Path B (or to re-download on Path A), download the model weights and test images:

```bash
# Download YOLO weights (extracts to packages/src/anytime_yolo/weights_32/)
cd packages/src/anytime_yolo
wget https://tu-dortmund.sciebo.de/s/gmGSJEsFgwKb6MY/download -O weights.zip
unzip -o weights.zip
rm weights.zip
cd ../../..

# Download test images (extracts to packages/src/video_publisher/images/)
cd packages/src/video_publisher
wget https://tu-dortmund.sciebo.de/s/BQRaiztJkmx33tt/download -O images.zip
unzip -o images.zip
rm images.zip
cd ../../../
```

> **First run after downloading weights:** The YOLO weights are distributed as ONNX models. On first use, TensorRT compiles each ONNX model into an optimized `.engine` file for your specific GPU. This one-time compilation adds **10–20 minutes** to the first YOLO experiment run. Subsequent runs reuse the cached `.engine` files and start immediately.

## Experiment Details

### Monte Carlo (Figures 5a, 5b)

**What it does:** Launches a ROS 2 anytime action server that computes Monte Carlo batches and a client that periodically sends goals and cancels them after 200 ms. Each configuration runs for a fixed duration while LTTng traces record batch computation times, cancellation delays, and throughput. The evaluation script parses the traces and plots segment counts and cancellation delays across batch sizes and modes.

**Why it takes this long:** Each of the 28 (full) or 12 (quick) configurations runs sequentially for the configured duration, plus ~10 s overhead per config for LTTng session setup/teardown and node startup/cleanup. Trace parsing and plot generation add a few more minutes at the end.

| Parameter | Full | Quick |
| --------- | ---- | ----- |
| Batch sizes | 1024, 2048, 4096, 8192, 16384, 32768, 65536 | 1024, 16384, 65536 |
| Modes | reactive, proactive | reactive, proactive |
| Threading | single, multi | single, multi |
| Run duration | 10 seconds per config | 5 seconds per config |
| Total configs | 28 | 12 |
| **Estimated time** | **~40 min** | **~5 min** |

### Interference (Figure 6, Table I)

**What it does:** Runs the same Monte Carlo action server alongside a periodic interference timer node that performs a 10 ms busy-wait every 100 ms, both in a single-threaded ROS 2 executor. This creates CPU contention: larger batch sizes block the executor longer, causing the timer to miss its 100 ms period. LTTng traces record timer callback timestamps and compute batch durations. The evaluation script measures timer jitter (deviation from the expected 100 ms period), missed-period rates, and compute times per batch size.

**Why it takes this long:** 14 (full) or 6 (quick) configurations run sequentially. Each config has the same per-config overhead as Monte Carlo (~10 s for LTTng and node lifecycle), plus the run duration itself.

| Parameter | Full | Quick |
| --------- | ---- | ----- |
| Batch sizes | 1024, 2048, 4096, 8192, 16384, 32768, 65536 | 1024, 16384, 65536 |
| Modes | reactive, proactive | reactive, proactive |
| Threading | single | single |
| Timer period | 100 ms | 100 ms |
| Timer execution time | 10 ms | 10 ms |
| Run duration | 10 seconds per config | 5 seconds per config |
| Total configs | 14 | 6 |
| **Estimated time** | **~40 min** | **~3 min** |

### YOLO (Figures 7a, 7b)

**What it does:** Runs a YOLOv3 object detection pipeline split into 25 individually-executable neural network layers on the GPU using TensorRT. The experiment proceeds in phases:

1. **Baseline collection (Step 1):** Processes 192 test images through all 25 layers one at a time (batch size 1) over 3 trials, recording per-layer computation times and detection quality via LTTng.
2. **Quality analysis (Step 2a):** Parses baseline traces to determine how detection quality progresses as more layers execute — showing at which layer the model reaches 50 %, 70 %, 90 % of final quality. Produces Figure 7a.
3. **Throughput measurement (Step 3):** Processes all 192 images with batch size 25 (all layers at once) across 4 configurations (sync/async × single/multi-threaded), measuring end-to-end throughput.
4. **Throughput analysis (Step 4):** Parses throughput traces and produces Figure 7b comparing images/second across configurations.

**Why it takes this long:** The baseline collection (3 trials × 192 images × 25 layers of GPU inference) dominates the time for Figure 7a. The throughput measurement (4 configurations × 192 images) adds another pass for Figure 7b. Trace parsing for thousands of layer-level events takes additional minutes per phase.

| Parameter | Value |
| --------- | ----- |
| Network | YOLOv3 with 25 layers |
| Test images | 192 |
| Block sizes | 1, 8, 16, 25 |
| Sync modes | sync, async |
| Threading | single, multi |
| Mode | proactive |
| Baseline trials | 3 |
| Cancellation: cancel_after_layers | 25 |
| Cancellation: score_threshold | 0.7 |
| **Estimated time (Figure 7a)** | **~30 min** |
| **Estimated time (Figure 7b)** | **~1 hr** |
| **Estimated time (both)** | **~1.5 hrs** |

## Environment Details

The Docker images provide a complete, self-contained environment:

- **OS:** Ubuntu 22.04 (Jammy)
- **ROS 2:** Humble Hawksbill (built from source)
- **DDS:** CycloneDDS (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`)
- **Build system:** colcon
- **Tracing:** LTTng (lttng-tools, liblttng-ust-dev, python3-babeltrace)
- **GPU image (Path B):** CUDA 12.5, TensorRT 10.3.0
- **Jetson image (Path A):** NVIDIA JetPack 6 (L4T r36.4.0), CUDA 12.6, TensorRT 10.3.0
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

**SSH connection refused or timed out (Path A):**

Verify you are using the correct IP, port, username, and password from the confidential credentials document. Ensure you are connected to a network that allows outbound SSH on the specified port. If the connection still fails, contact the authors.

**Docker container already running on the Jetson (Path A):**

If another reviewer's container is still running, you may see a port conflict or resource contention. Check with:

```bash
docker ps
```

If a container is running, wait for the other reviewer to finish, or coordinate access. Do not run experiments concurrently, as this affects timing results.

**CUDA initialization failure (error 35) during YOLO warmup:**

This error means the TensorRT version inside the container is incompatible with the installed CUDA toolkit or your host GPU driver. The container pins TensorRT 10.3.0 for CUDA 12.5, which requires NVIDIA driver 535 or newer. Check your host driver version:

```bash
nvidia-smi   # Look for "Driver Version" — must be 535+
```

If your driver is older, update it. If the container was built before the TensorRT version was pinned, rebuild it:

```bash
docker compose build --no-cache anytime-gpu
```

**TensorRT engines rebuilding on first run or after switching GPUs:**

TensorRT `.engine` files are compiled for a specific GPU architecture. When you run experiments on a different GPU than the one that previously built the engines, the system automatically detects the mismatch via a GPU fingerprint, removes the stale engines, and rebuilds them from the source ONNX models. This one-time rebuild adds 10–20 minutes. To force a manual rebuild:

```bash
./scripts/warmup_yolo_engines.sh --force-rebuild
```

## Project Structure

```
Anytime-Development/
├── ARTIFACT_EVALUATION.md      # This guide
├── main.pdf                    # Accepted paper
├── docker-compose.yml          # Container management
├── paper_figures/              # Collected paper figures (generated)
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
    ├── reproduce_figure.sh     # Per-figure reproduction
    └── collect_figures.sh      # Collect paper figures into paper_figures/
```
