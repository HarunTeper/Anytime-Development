# Interference Experiments

Measure timing interference between Monte Carlo batch processing and a periodic timer task.

## Quick Start

```bash
# Test (10 seconds)
./test_single_config.sh

# Generate configs
python3 generate_configs.py

# Run full experiments (~40 min)
./run_interference_experiments.sh

# View results
cat results/aggregated_results.csv
ls results/plots/
```

## Configuration

**Monte Carlo:**
- Batch sizes: 1, 64, 4096, 16384, 65536, 262144
- Modes: reactive, proactive
- Threading: single, multi

**Timer (Fixed):**
- Period: 100ms (10 Hz)
- Execution: 10ms busy-wait

**Total**: 24 configs × 3 runs = 72 experiments

## Metrics

- Timer period jitter
- Missed timer periods (>150% expected)
- Compute batch timing
- Interference severity
│       ├── timer_period_vs_batch_size.png
│       ├── jitter_vs_batch_size.png
│       ├── missed_periods_percentage.png
│       ├── compute_time_vs_batch_size.png
│       └── timer_period_distribution.png
├── generate_configs.py         # Script to generate configuration files
├── run_interference_experiments.sh  # Main experiment execution script
├── evaluate_interference.py    # Analysis and plotting script
├── test_single_config.sh       # Quick test with one configuration
└── README.md                   # This file
```

## Quick Start

### 1. Generate Configuration Files

```bash
cd /home/vscode/workspace/experiments/interference
python3 generate_configs.py
```

This creates 72 YAML files (24 server + 24 client + 24 interference configs) in the `configs/` directory.

### 2. Test with a Single Configuration

Before running the full experiment suite, test with a single configuration:

```bash
chmod +x test_single_config.sh
./test_single_config.sh
```

This runs one configuration for 10 seconds to verify everything works.

### 3. Run All Experiments

```bash
chmod +x run_interference_experiments.sh
./run_interference_experiments.sh
```

This will:
- Run all 72 experiments (24 configs × 3 runs)
- Each run lasts 30 seconds
- Total time: ~40 minutes (including setup/teardown)
- Automatically call the evaluation script when done

### 4. Run Only Evaluation (if traces already exist)

```bash
python3 evaluate_interference.py
```

## Configuration Format

### Server Configuration
```yaml
anytime_server:
  ros__parameters:
    is_reactive_proactive: "reactive"  # or "proactive"
    multi_threading: true              # or false
    batch_size: 1                      # 1, 64, 4096, 16384, 65536, 262144
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

### Interference Timer Configuration
```yaml
interference_timer:
  ros__parameters:
    timer_period_ms: 100               # Timer period (fixed at 100ms)
    execution_time_ms: 10              # Busy-wait duration (fixed at 10ms)
    log_level: "info"
```

## Metrics Collected

### Primary Metrics (Timer Interference)

1. **Timer Period**: Time between consecutive timer callback starts
   - Expected: 100 ms
   - Deviation indicates interference from Monte Carlo

2. **Timer Jitter**: Deviation from expected 100ms period
   - `jitter = actual_period - expected_period`
   - Positive jitter = timer was delayed
   - Negative jitter = timer ran early (rare)

3. **Missed Periods**: Count of periods > 150ms (>50% deviation)
   - Indicates severe interference

4. **Timer Execution Time**: Time spent in timer callback
   - Expected: ~10 ms (busy-wait duration)

### Secondary Metrics (Monte Carlo Performance)

5. **Compute Batch Time**: Duration of each Monte Carlo compute batch
   - Shows how long each batch takes

6. **Total Compute Batches**: Number of batches completed in 30s

### Aggregation

- Metrics are averaged across the 3 runs for each configuration
- Standard deviations are computed for variability analysis

## Generated Plots

1. **timer_period_vs_batch_size.png**: 
   - Shows average timer period vs. batch size
   - Compares all configurations (reactive/proactive, single/multi-threaded)
   - Includes expected 100ms reference line
   - **Key plot**: Shows interference effect clearly

2. **jitter_vs_batch_size.png**: 
   - Shows maximum absolute jitter vs. batch size
   - Log-log scale to show growth trends
   - Higher values = more interference

3. **missed_periods_percentage.png**: 
   - Bar chart showing percentage of missed periods
   - Periods > 150% of expected (>150ms)
   - Direct measure of severe interference

4. **compute_time_vs_batch_size.png**: 
   - Shows Monte Carlo compute batch timing
   - Explains why larger batches cause more interference

5. **timer_period_distribution.png**: 
   - Box plots showing timer period distributions
   - Separate subplots for each configuration type
   - Shows variability and outliers

## Output Files

### CSV Files
- `individual_runs.csv`: Raw metrics from all 72 runs
- `aggregated_results.csv`: Averaged metrics for each of 24 configurations

### JSON File
- `aggregated_results.json`: Complete results in JSON format for further processing

## Tracepoints Used

The experiments rely on these LTTng tracepoints:

### Interference Timer Events (Primary)
- `anytime:interference_timer_init` - Timer initialization
- `anytime:interference_timer_callback_entry` - Timer callback start (used to measure periods)
- `anytime:interference_timer_callback_exit` - Timer callback end (used to measure execution time)

### Monte Carlo Events (Secondary)
- `anytime:anytime_compute_entry` - Compute batch start
- `anytime:anytime_compute_exit` - Compute batch end

**Note**: We do NOT trace individual Monte Carlo iterations to reduce overhead and trace size.

## Expected Results

### Small Batch Sizes (1, 64)
- Timer period ≈ 100ms
- Low jitter (<5ms)
- No missed periods (0%)
- Minimal interference

### Medium Batch Sizes (4096, 16384)
- Timer period starts deviating (100-120ms)
- Moderate jitter (5-20ms)
- Few missed periods (<10%)
- Noticeable interference

### Large Batch Sizes (65536, 262144)
- Timer period significantly delayed (>150ms)
- High jitter (>50ms)
- Many missed periods (>30%)
- Severe interference

### Threading Comparison
- **Single-threaded**: More interference (timer and compute compete)
- **Multi-threaded**: Less interference (timer can run concurrently)

### Mode Comparison
- **Reactive**: May show different patterns due to cancellation handling
- **Proactive**: May show different patterns due to continued computation

## Customization

### Modify Experiment Duration
Edit `run_interference_experiments.sh`:
```bash
RUN_DURATION=30  # Change to desired duration in seconds
```

### Modify Number of Runs
Edit `run_interference_experiments.sh`:
```bash
NUM_RUNS=3  # Change to desired number of runs per config
```

### Modify Batch Sizes
Edit `generate_configs.py`:
```python
batch_sizes = [1, 64, 4096, 16384, 65536, 262144]  # Add or remove values
```
Then regenerate configs:
```bash
python3 generate_configs.py
```

### Modify Timer Parameters
Edit `generate_configs.py`:
```python
TIMER_PERIOD_MS = 100  # Timer period (ms)
EXECUTION_TIME_MS = 10  # Timer busy-wait duration (ms)
```
Then regenerate configs and update `run_interference_experiments.sh` to match.

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
- Check if tracepoints are available: `lttng list --userspace | grep interference`
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
- Verify packages are built: `cd /home/vscode/workspace/packages && colcon build`
- Source the workspace: `source /home/vscode/workspace/packages/install/setup.bash`
- Check config files exist: `ls configs/`

### Interference timer not running
- Check that `interference` package is built
- Verify tracepoints exist: `lttng list --userspace | grep interference_timer`

## Expected Runtime

- Single configuration test: ~15 seconds
- Full experiment suite (72 runs × 30s): ~40-45 minutes
- Evaluation script: ~1-2 minutes

## Analysis Tips

1. **Look for trends in timer period vs batch size**
   - Does period increase linearly? Exponentially?
   - At what batch size does interference become significant?

2. **Compare single vs multi-threaded**
   - Multi-threaded should show less interference
   - Quantify the benefit of multi-threading

3. **Compare reactive vs proactive**
   - Does mode affect interference patterns?
   - Consider cancellation vs continued computation effects

4. **Analyze missed periods**
   - What percentage is acceptable for your application?
   - At what batch size does it become problematic?

## Related Experiments

This experiment complements the Monte Carlo experiment:
- **monte_carlo/**: Focuses on Monte Carlo performance metrics
- **interference/**: Focuses on how Monte Carlo affects other tasks

Together, they provide a complete picture of anytime algorithm behavior.

## Contact

For questions or issues with the experimental setup, please refer to the main project documentation.
