# Anytime ROS2 - Experimental Evaluation

ROS2 implementation of anytime algorithms with comprehensive tracing and experimental evaluation.

## Quick Start

### Setup
1. Install [Docker Engine](https://docs.docker.com/engine/install/) and [VS Code](https://code.visualstudio.com/)
2. Install Docker and Dev Containers extensions in VS Code
3. For GPU support: Install [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
4. Open this folder in VS Code and rebuild the dev container

### Build Workspace
```bash
cd /home/vscode/workspace/packages
colcon build --symlink-install
source install/setup.bash
```

## Experiments

Three experimental evaluations are implemented:

### 1. Monte Carlo (`experiments/monte_carlo/`)
**Purpose:** Evaluate batch size scaling and threading impact  
**Run:** `./run_monte_carlo_experiments.sh`  
**Duration:** ~40 minutes (24 configs × 3 trials)

### 2. Interference (`experiments/interference/`)
**Purpose:** Measure timing interference between batch processing and periodic tasks  
**Run:** `./run_interference_experiments.sh`  
**Duration:** ~40 minutes (24 configs × 3 trials)

### 3. YOLO (`experiments/yolo/`)
**Purpose:** Evaluate anytime YOLO with cancellation strategies  
**Phases:**
- Phase 1: `./run_phase1_baseline.sh` - Baseline quality data
- Phase 3: `./run_phase3_max_throughput.sh` - Max throughput across configs
- Phase 4: `./run_phase4_experiments.sh` - Cancellation performance

See individual experiment directories for detailed documentation.

## Tracing

All experiments use LTTng for low-overhead tracing. Custom tracepoints are defined in `packages/src/anytime_tracing/`.

**Key tracepoints:**
- `anytime:anytime_compute_entry/exit` - Batch computation timing
- `anytime:client_send_goal` - Client request tracking
- `anytime:yolo_layer_start/end` - YOLO layer processing
- `anytime:interference_timer_callback` - Timer interference

**View traces:**
```bash
babeltrace ~/.lttng-traces/session_name/
```

## Project Structure

```
packages/src/
├── anytime_core/          # Base anytime functionality
├── anytime_tracing/       # LTTng tracing infrastructure
├── anytime_monte_carlo/   # Monte Carlo implementation
├── anytime_yolo/          # Anytime YOLO implementation
├── interference/          # Timer interference test node
└── experiments/           # Unified launch files and configs

experiments/
├── monte_carlo/           # Monte Carlo experiments
├── interference/          # Interference experiments
└── yolo/                  # YOLO experiments
```