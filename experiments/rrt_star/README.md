# RRT* Experiments

Evaluate RRT* path planning as an anytime workload under the Anytime framework, measuring batch-size scaling, threading impact, and path cost convergence on Nav2 occupancy grid maps.

## Quick Start

```bash
# 1. Build the workspace
cd packages && colcon build --packages-select anytime_rrt_star && source install/setup.bash && cd ..

# 2. Generate experiment configs
python3 experiments/rrt_star/generate_configs.py

# 3. Quick smoke test (~1 min, depot map only)
./experiments/rrt_star/run_quick.sh

# 4. Full experiment suite
./experiments/rrt_star/run_rrt_star_experiments.sh

# 5. Re-run evaluation only (if traces already exist)
python3 experiments/rrt_star/evaluate_rrt_star.py

# 6. View results
cat experiments/rrt_star/results/aggregated_results.csv
ls experiments/rrt_star/results/plots/
```

## Scripts

| Script | Purpose |
|---|---|
| `generate_configs.py` | Generate all server/client YAML configs for the experiment matrix |
| `run_rrt_star_experiments.sh` | Run the full experiment suite with LTTng tracing |
| `run_quick.sh` | Quick subset run for development/CI (depot, single-threaded, 3 batch sizes) |
| `test_single_config.sh` | Test a single configuration interactively |
| `evaluate_rrt_star.py` | Parse LTTng traces, compute metrics, export CSV/JSON, generate plots |
| `visualize_maps.py` | Render maps with start/goal markers overlaid |

## Configuration

The experiment matrix covers:

- **Batch sizes**: 1, 16, 64, 256, 1024, 4096, 16384
- **Modes**: reactive, proactive
- **Threading**: single, multi
- **Maps**: depot, warehouse
- **Runs per config**: 5 (configurable)
- **Run duration**: 10 seconds per config (configurable)
- **Total**: 560 runs (112 configurations × 5 runs)

### Maps

Two Nav2 occupancy grid maps are used:

- **depot** — 604×307 px, 0.05 m/px (30.2×15.4 m). Small, mostly open layout with scattered obstacles.
  - Start: (5.0, 12.0), Goal: (25.0, 2.0)
- **warehouse** — 1006×1674 px, 0.03 m/px (30.2×50.2 m). Large structured environment with aisles.
  - Start: (−12.5, −20.0), Goal: (12.5, 20.0)

### Visualizing Maps

Render the maps with start/goal markers to verify positions:

```bash
# PNG output (default) to results/plots/
python3 experiments/rrt_star/visualize_maps.py

# PDF output
python3 experiments/rrt_star/visualize_maps.py --format pdf

# Custom output directory
python3 experiments/rrt_star/visualize_maps.py --output-dir /tmp/map_plots

# Custom maps directory
python3 experiments/rrt_star/visualize_maps.py --maps-dir /path/to/maps
```

Map visualizations are also automatically generated as part of `evaluate_rrt_star.py`.

## Test a Single Configuration

```bash
# Default: batch_size=256, reactive, single-threaded, depot, 10s
./experiments/rrt_star/test_single_config.sh

# Custom parameters: batch_size mode threading map duration
./experiments/rrt_star/test_single_config.sh 1024 proactive multi warehouse 15
```

## Metrics Collected

### Framework Metrics
- **Time per batch** — Duration of each batch computation (ms)
- **Throughput** — Iterations per second
- **Cancellation delay** — Time from cancel request to deactivation (ms)
- **Goal-to-finish latency** — Round-trip time from goal sent to result received (ms)
- **Goal-to-cancel latency** — Time from goal sent to cancel sent (ms)
- **Cancel-to-finish latency** — Time from cancel sent to result received (ms)

### Overhead Metrics
- **Per-batch overhead** — Gap between consecutive compute batches (ms)
- **Overhead ratio** — Overhead as percentage of total cycle time
- **Feedback send time** — Time spent sending feedback to client (ms)
- **Result compute time** — Time spent calculating final result (ms)
- **Batch time percentiles** — p50, p95, p99 of batch compute times

### RRT*-Specific Metrics
- **Convergence curves** — Best path cost vs. iteration number
- **First solution iteration** — When a feasible path is first found
- **Best cost at cancellation** — Final path cost when cancelled
- **Tree size vs. iterations** — Node count growth over time

## Generated Plots

Plots are organized into subdirectories under `results/plots/`.

### Framework Plots (`results/plots/framework/`)
- `batch_size_vs_time[_<map>].pdf` — Batch compute time by configuration
- `throughput[_<map>].pdf` — Iterations per second
- `server_cancel_response[_<map>].pdf` — Server cancel response delay
- `goal_to_finish_latency.pdf` — Round-trip latency
- `total_cancellation_time.pdf` — Combined cancel latency
- `total_iterations.pdf` — Iterations completed per run

### Overhead Plots (`results/plots/overhead/`)
- `per_batch_overhead[_<map>].pdf` — Inter-batch overhead
- `overhead_ratio[_<map>].pdf` — Overhead percentage
- `feedback_send_time[_<map>].pdf` — Feedback timing
- `result_compute_time[_<map>].pdf` — Result calculation timing
- `batch_time_percentiles[_<map>].pdf` — p50/p95/p99 distributions
- `batch_time_trend[_<map>].pdf` — Batch time over run duration

### RRT*-Specific Plots (`results/plots/rrt_star/`)
- `convergence_curve_<map>.pdf` — Cost convergence per map
- `convergence_by_map.pdf` — Cross-map convergence comparison
- `first_solution_iteration_<map>.pdf` — When first path is found
- `best_cost_vs_batch_size[_<map>].pdf` — Final cost quality
- `tree_size_vs_iterations_<map>.pdf` — Tree growth

### Map Visualizations (`results/plots/maps/`)
- `map_depot.{png,pdf}` — Depot map with start/goal markers
- `map_warehouse.{png,pdf}` — Warehouse map with start/goal markers

## Output Files

```
experiments/rrt_star/
├── configs/                        # Generated YAML configs
│   ├── batch_1_reactive_single_depot_server.yaml
│   ├── batch_1_reactive_single_depot_client.yaml
│   └── ...
├── traces/                         # LTTng trace data (per run)
│   ├── batch_256_reactive_single_depot_run1/
│   └── ...
└── results/
    ├── individual_runs.csv         # Raw metrics from each run
    ├── aggregated_results.csv      # Averaged metrics per config
    ├── aggregated_results.json     # Full results in JSON
    ├── convergence_data/           # Per-map convergence CSVs
    │   ├── depot_convergence.csv
    │   └── warehouse_convergence.csv
    └── plots/
        ├── legend.pdf              # Shared legend
        ├── framework/              # Framework metric plots
        ├── overhead/               # Overhead metric plots
        ├── rrt_star/               # RRT*-specific plots
        └── maps/                   # Map visualizations
```

## Customization

### Modify Experiment Duration

Edit `run_rrt_star_experiments.sh`:
```bash
RUN_DURATION=10  # seconds per configuration
```

### Modify Batch Sizes

Edit `generate_configs.py`:
```python
batch_sizes = [1, 16, 64, 256, 1024, 4096, 16384]
```
Then regenerate: `python3 generate_configs.py`

### Modify Number of Runs

Edit `run_rrt_star_experiments.sh`:
```bash
NUM_RUNS=5  # trials per configuration
```

## Tracepoints Used

| Tracepoint | Purpose |
|---|---|
| `anytime:anytime_compute_entry/exit` | Batch computation boundaries |
| `anytime:anytime_send_feedback_entry/exit` | Feedback send timing |
| `anytime:anytime_calculate_result_entry/exit` | Result calculation timing |
| `anytime:anytime_server_handle_cancel` | Server-side cancel receipt |
| `anytime:anytime_base_deactivate` | Computation deactivation |
| `anytime:anytime_client_goal_sent` | Client goal dispatch |
| `anytime:anytime_client_cancel_sent` | Client cancel dispatch |
| `anytime:anytime_client_goal_finished` | Client result receipt |
| `anytime:rrt_star_iteration` | Per-iteration RRT* state (subsampled) |
| `anytime:rrt_star_result` | Final RRT* result |
| `anytime:rrt_star_init` | RRT* initialization |
| `anytime:rrt_star_reset` | RRT* tree reset |

## Dependencies

- ROS 2 Humble (with `anytime_rrt_star` and `experiments` packages built)
- LTTng tools (`lttng-tools`, `lttng-modules-dkms`, `liblttng-ust-dev`)
- babeltrace (for trace parsing)
- Python 3 with: `pandas`, `numpy`, `matplotlib`

```bash
# Install Python dependencies
pip3 install pandas numpy matplotlib

# Install LTTng + babeltrace (Ubuntu)
sudo apt-get install lttng-tools liblttng-ust-dev babeltrace
```

## Troubleshooting

### No trace data generated
- Verify LTTng is installed: `lttng --version`
- Check tracepoints are available: `lttng list --userspace | grep anytime`
- Ensure workspace is sourced before running experiments

### babeltrace not found
```bash
sudo apt-get install babeltrace
```

### Experiments fail to launch
- Verify packages are built: `cd packages && colcon build`
- Source the workspace: `source packages/install/setup.bash`
- Check config files exist: `ls experiments/rrt_star/configs/`
