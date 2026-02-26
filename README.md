# Anytime ROS 2: Timely Task Completion in Non-Preemptive Robotic Systems

This repository contains the artifact for the paper *"Anytime ROS 2: Timely Task Completion in Non-Preemptive Robotic Systems"*, accepted at **IEEE RTAS 2026**. It provides a ROS 2 implementation of anytime algorithms along with all experiment scripts and evaluation code needed to reproduce the paper results.

**Authors:** Harun Teper, Daniel Kuhse, Yun-Chih Chen, Georg von der Brüggen, Zhishan Guo, Jian-Jia Chen
**Contact:** harun.teper (at) tu-dortmund.de

> **Artifact Evaluation:** See [`ARTIFACT_EVALUATION.md`](ARTIFACT_EVALUATION.md) for step-by-step instructions to reproduce all paper figures and tables.

## Quick Start

### Prerequisites

- [Docker Engine](https://docs.docker.com/engine/install/) (24+) with Compose V2
- For GPU support: [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

### Build and Run

```bash
# CPU-only
docker compose build anytime-cpu
docker compose run --rm anytime-cpu bash

# OR with GPU support
docker compose build anytime-gpu
docker compose run --rm anytime-gpu bash

# OR on NVIDIA Jetson (ARM64)
docker compose build anytime-jetson
docker compose run --rm anytime-jetson bash
```

Inside the container:

```bash
cd packages && colcon build --symlink-install && source install/setup.bash
cd ..

# Run smoke test (~2 minutes)
./scripts/smoke_test.sh

# Run all CPU experiments with quick settings (~5 minutes)
./scripts/run_all.sh --quick --cpu-only

# Full reproduction including YOLO (~1.5 hours)
./scripts/run_all.sh --full

# Run specific experiment groups
./scripts/run_all.sh --full --monte-carlo
./scripts/run_all.sh --full --interference
./scripts/run_all.sh --full --yolo

# Reproduce a specific paper figure
./scripts/reproduce_figure.sh 5a --quick
```

### VS Code Dev Container

Alternatively, open this folder in VS Code with the Dev Containers extension. Three configurations are available:

- `linux/` — Full environment with CUDA + TensorRT (GPU required)
- `linux-no-hardware/` — CPU-only environment
- `jetson/` — NVIDIA Jetson (ARM64)

## Experiments

Three experimental evaluations reproduce the paper results:

### Monte Carlo (Figures 5a, 5b)

Evaluates batch size scaling, mode comparison (reactive vs proactive), and threading impact on anytime Monte Carlo pi estimation.

```bash
./scripts/reproduce_figure.sh 5a        # Full run (~10 min)
./scripts/reproduce_figure.sh 5a --quick # Quick run (~3 min)
./scripts/reproduce_figure.sh 5 --quick  # Both 5a + 5b (~3 min)
```

### Interference (Figures 6a, 6b, Table I)

Measures timing interference between anytime batch processing and periodic timer tasks.

```bash
./scripts/reproduce_figure.sh 6          # Full run (~7 min)
./scripts/reproduce_figure.sh 6 --quick  # Quick run (~3 min)
```

### YOLO (Figures 7a, 7b) — GPU Required

Evaluates anytime YOLO object detection with layer-wise cancellation on GPU.

```bash
./scripts/reproduce_figure.sh 7a  # Quality progression (~15 min)
./scripts/reproduce_figure.sh 7b  # Runtime comparison (~25 min)
./scripts/reproduce_figure.sh 7   # Both 7a + 7b (~35 min)
```

See [`ARTIFACT_EVALUATION.md`](ARTIFACT_EVALUATION.md) for YOLO prerequisites (weights and test images).

### Output

Paper figures are collected into `paper_figures/` after each run. To re-collect at any time:

```bash
./scripts/collect_figures.sh
```

### Cleanup

To re-run experiments from a clean state:

```bash
./scripts/run_all.sh --clean      # Remove experiment outputs
./scripts/run_all.sh --clean-all  # Also remove build artifacts, YOLO weights, and video frames
```

## Tracing

All experiments use LTTng for low-overhead tracing. Custom tracepoints are defined in `packages/src/anytime_tracing/`.

Key tracepoints:

- `anytime:anytime_compute_entry/exit` — Batch computation timing
- `anytime:anytime_client_send_goal` — Client request tracking
- `anytime:yolo_layer_start/end` — YOLO layer processing
- `anytime:interference_timer_callback_entry/exit` — Timer interference

View traces:

```bash
babeltrace ~/.lttng-traces/session_name/
```

## Project Structure

```text
Anytime-Development/
├── ARTIFACT_EVALUATION.md      # Artifact evaluation guide
├── docker-compose.yml          # Container management (GPU + CPU + Jetson)
├── LICENSE                     # Apache 2.0
├── requirements.txt            # Python dependencies
├── paper_figures/              # Collected paper figures (generated)
├── packages/src/
│   ├── anytime_core/           # Base anytime computation framework
│   ├── anytime_interfaces/     # ROS 2 action type definitions
│   ├── anytime_monte_carlo/    # Monte Carlo pi estimation (CPU)
│   ├── anytime_yolo/           # Anytime YOLO object detection (GPU)
│   ├── anytime_tracing/        # LTTng tracepoint definitions
│   ├── experiments/            # Launch files and default configs
│   ├── interference/           # Timer interference test node
│   ├── test_action/            # Test action for executor testing
│   └── video_publisher/        # Video frame publisher for YOLO
├── experiments/
│   ├── monte_carlo/            # MC experiment scripts and evaluation
│   ├── interference/           # Interference experiment scripts and evaluation
│   └── yolo/                   # YOLO pipeline (9 steps)
└── scripts/
    ├── smoke_test.sh           # Quick validation (< 2 min)
    ├── run_all.sh              # End-to-end runner (--quick/--full/--cpu-only/--gpu-only/--clean)
    ├── reproduce_figure.sh     # Per-figure reproduction
    ├── collect_figures.sh      # Collect paper figures into paper_figures/
    └── warmup_yolo_engines.sh  # Build TensorRT engines from ONNX weights
```

## Citation

```bibtex
@inproceedings{teper2026anytime,
  title     = {Anytime {ROS} 2: Timely Task Completion in Non-Preemptive Robotic Systems},
  author    = {Teper, Harun and Kuhse, Daniel and Chen, Yun-Chih and von der Br{\"u}ggen, Georg and Guo, Zhishan and Chen, Jian-Jia},
  booktitle = {IEEE Real-Time and Embedded Technology and Applications Symposium (RTAS)},
  year      = {2026}
}
```

## License

This project is licensed under the Apache License 2.0. See [`LICENSE`](LICENSE) for details.
