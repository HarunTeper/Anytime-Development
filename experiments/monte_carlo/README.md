# Monte Carlo Experiments

Evaluate Monte Carlo block size scaling and threading impact.

## Quick Start

```bash
# Test (10 seconds)
./test_single_config.sh

# Generate configs
python3 generate_configs.py

# Run full experiments (~10 min)
./run_monte_carlo_experiments.sh

# View results
cat results/aggregated_results.csv
ls results/plots/
```

## Configuration

- **Block sizes**: 1024, 2048, 4096, 8192, 16384, 32768, 65536
- **Modes**: reactive, proactive
- **Threading**: single, multi
- **Total**: 28 configs × 5 runs = 140 experiments

## Metrics

- Iterations per block
- Time per block
- Throughput (iterations/second)
- Cancellation delay

### 3. Run All Experiments

```bash
./run_monte_carlo_experiments.sh
```

This will:
- Run all 140 experiments (28 configs × 5 runs)
- Each run lasts 10 seconds
- Total time: ~30 minutes (including setup/teardown)
- Automatically call the evaluation script when done

### 4. Run Only Evaluation (if traces already exist)

```bash
python3 evaluate_monte_carlo.py
```

## Configuration Format

### Server Configuration
```yaml
anytime_server:
  ros__parameters:
    is_reactive_proactive: "reactive"  # or "proactive"
    multi_threading: true              # or false
    block_size: 1024                   # 1024, 2048, 4096, 8192, 16384, 32768, 65536
    log_level: "info"
```

### Client Configuration
```yaml
anytime_client:
  ros__parameters:
    goal_timer_period_ms: 1000         # Send goal every 1 second
    cancel_timeout_period_ms: 250      # Cancel after 250ms
    log_level: "info"
```

## Metrics Collected

The evaluation script extracts and analyzes:

### Per-Block Metrics
- **Iterations per block**: Number of Monte Carlo iterations in each block
- **Time per block**: Duration of each block computation (ms)
- **Compute time**: Time spent in actual computation
- **Feedback time**: Time spent sending feedback to client
- **Result time**: Time spent calculating final result

### Overall Metrics
- **Total blocks completed**: Number of blocks finished in 10s
- **Total Segments**: Total Monte Carlo iterations executed
- **Cancellation delay**: Time from cancel request to deactivation (ms)
- **Throughput**: Iterations per second

### Aggregation
- Metrics are averaged across the 5 runs for each configuration
- Standard deviations are computed for variability analysis

## Generated Plots

1. **block_size_vs_iterations.png**: Shows how many iterations are completed per block for different block sizes
2. **block_size_vs_time.png**: Shows computation time per block vs. block size
3. **server_cancel_response.pdf**: Compares server cancel response delays across configurations
4. **threading_comparison.png**: Compares single vs. multi-threaded performance
5. **throughput.png**: Overall throughput (iterations/second) for each configuration

All plots compare:
- Reactive vs. Proactive modes
- Single-threaded vs. Multi-threaded executors

## Output Files

### CSV Files
- `individual_runs.csv`: Raw metrics from each of the 140 runs
- `aggregated_results.csv`: Averaged metrics for each of 28 configurations

### JSON File
- `aggregated_results.json`: Complete results in JSON format for further processing

## Customization

### Modify Experiment Duration
Edit `run_monte_carlo_experiments.sh`:
```bash
RUN_DURATION=10  # Change to desired duration in seconds
```

### Modify Number of Runs
Edit `run_monte_carlo_experiments.sh`:
```bash
NUM_RUNS=5  # Change to desired number of runs per config
```

### Modify Block Sizes
Edit `generate_configs.py`:
```python
block_sizes = [1024, 2048, 4096, 8192, 16384, 32768, 65536]  # Add or remove values
```
Then regenerate configs:
```bash
python3 generate_configs.py
```

## Tracepoints Used

The experiments rely on these LTTng tracepoints:
- `anytime:anytime_compute_entry/exit` - Block boundaries
- `anytime:anytime_compute_iteration` - Individual iterations
- `anytime:monte_carlo_iteration` - Monte Carlo specific iterations
- `anytime:anytime_send_feedback_entry/exit` - Feedback timing
- `anytime:anytime_calculate_result_entry/exit` - Result calculation timing
- `anytime:anytime_server_handle_cancel` - Cancellation requests
- `anytime:anytime_base_deactivate` - Computation deactivation

## Dependencies

- ROS 2 (with experiments package built)
- LTTng tools (`lttng-tools`, `lttng-modules-dkms`, `liblttng-ust-dev`)
- Python 3 with:
  - pandas
  - numpy
  - matplotlib
- babeltrace (for parsing traces)

## Troubleshooting

### No trace data generated
- Verify LTTng is installed: `lttng --version`
- Check if tracepoints are available: `lttng list --userspace | grep anytime`
- Ensure workspace is sourced before running experiments

### babeltrace not found
```bash
sudo apt-get install babeltrace
```

### Python dependencies missing
```bash
pip3 install pandas numpy matplotlib
```

### Experiments fail to launch
- Verify packages are built: `cd packages && colcon build`
- Source the workspace: `source packages/install/setup.bash`
- Check config files exist: `ls configs/`

## Expected Runtime

- Single configuration test: ~15 seconds
- Full experiment suite (140 runs × 10s): ~30 minutes
- Evaluation script: ~1-2 minutes

## Contact

For questions or issues with the experimental setup, please refer to the main project documentation.
