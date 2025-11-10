# YOLO Experiments

Evaluate Anytime YOLO performance with different cancellation strategies and configurations.

## Quick Start

```bash
# Test setup
./test_single_config.sh

# Phase 1: Baseline (collect quality data)
./run_phase1_baseline.sh

# Phase 3: Max throughput (4 configs × 3 trials)
./run_phase3_max_throughput.sh

# Phase 4: Cancellation performance (24 configs × 3 trials)
./run_phase4_experiments.sh

# Analyze results
python3 evaluate_yolo.py
python3 analyze_quality.py
python3 analyze_runtime.py
python3 analyze_phase4.py
```

## Phases

### Phase 1: Baseline Quality
- Collect layer-wise detection data
- Single-threaded, batch_size=1
- No cancellation (all 25 layers)

### Phase 3: Maximum Throughput
- Test 4 configurations: sync/async × single/multi-threaded
- batch_size=25 (all layers at once)
- Compare throughput and layer processing times

### Phase 4: Cancellation Performance
- 24 configurations: 3 block sizes × 2 modes × 2 sync modes × 2 threading
- Cancel after 16 layers or score threshold 0.8
- Measure cancellation delay and total runtime

## Analysis Scripts

- `analyze_quality.py` - Quality progression analysis
- `analyze_runtime.py` - Layer computation timing
- `analyze_phase4.py` - Cancellation performance
- `analyze_blocks.py` - Block size impact
   ```bash
   cd /home/vscode/workspace/packages
   colcon build --symlink-install
   ```

2. **Download YOLO weights**:
   ```bash
   # Run the download task or use:
   cd /home/vscode/workspace/packages/src/anytime_yolo
   wget https://tu-dortmund.sciebo.de/s/W86QE9hUscsUPeM/download -O weights.zip
   unzip -o weights.zip -d .
   rm weights.zip
   ```

3. **Download test images**:
   ```bash
   # Run the download task or use:
   mkdir -p /home/vscode/workspace/packages/src/video_publisher/images
   cd /home/vscode/workspace/packages/src/video_publisher/images
   wget https://tu-dortmund.sciebo.de/s/aA9MDhgN2lBmeZk/download -O images.zip
   unzip -o images.zip -d .
   rm images.zip
   ```

4. **Install LTTng** (should already be installed in dev container):
   ```bash
   sudo apt-get update
   sudo apt-get install lttng-tools lttng-modules-dkms liblttng-ust-dev
   ```

5. **Install Python dependencies**:
   ```bash
   pip3 install pandas numpy matplotlib
   ```

## Running Experiments

### Phase 1: Baseline

Collect layer-wise detection data:

```bash
cd /home/vscode/workspace/experiments/yolo
./run_phase1_baseline.sh
```

This will:
- Run 3 trials of the baseline configuration
- Process all images with batch_size=1
- Collect traces for each trial
- Save traces to `traces/phase1_baseline_trial1/`, `trial2/`, `trial3/`

### Phase 3: Maximum Throughput

Measure maximum throughput:

```bash
cd /home/vscode/workspace/experiments/yolo
./run_phase3_max_throughput.sh
```

This will:
- Run 3 trials of the max throughput configuration
- Process all images with batch_size=25
- Collect traces for each trial
- Save traces to `traces/phase3_max_throughput_trial1/`, `trial2/`, `trial3/`

## Analyzing Results

After running experiments, evaluate the results:

```bash
cd /home/vscode/workspace/experiments/yolo
python3 evaluate_yolo.py
```

This will:
- Parse all trace directories
- Calculate metrics (runtime, throughput, layer times)
- Generate plots in `results/plots/`
- Export summary CSV and detailed JSON

## Configuration Files

Configuration files are in YAML format and specify:
- **experiment**: name, phase, description
- **server_params**: batch_size, mode (reactive/proactive), sync/async
- **client_params**: cancellation settings
- **video_publisher**: image path
- **trace**: LTTng session configuration

## Metrics Collected

### Per-Goal Metrics
- Total runtime (ms)
- Number of detections
- Layers processed

### Layer-wise Metrics
- Processing time per layer (ms)
- Detection count after each layer
- Exit calculation time

### Summary Metrics
- Average runtime
- Throughput (images/second)
- Average detections per image
- Layer timing statistics

## Tracepoints Used

The following LTTng tracepoints are collected:
- `anytime:yolo_init` - Server initialization
- `anytime:yolo_layer_start` - Layer computation start
- `anytime:yolo_layer_end` - Layer computation end
- `anytime:yolo_exit_calculation_start` - Exit calculation start
- `anytime:yolo_exit_calculation_end` - Exit calculation end (with detection count)
- `anytime:yolo_result` - Final result with total detections
- `anytime:anytime_base_activate` - Goal start
- `anytime:anytime_base_reset` - Goal reset

## Troubleshooting

### LTTng Session Already Exists
```bash
lttng destroy yolo_phase1_baseline
# or
lttng destroy yolo_phase3_max_throughput
```

### No Images Found
Check that images are downloaded to:
```
/home/vscode/workspace/packages/src/video_publisher/images/
```

### YOLO Weights Not Found
Check that weights are downloaded to:
```
/home/vscode/workspace/packages/src/anytime_yolo/weights_32/
```

### Permission Denied on Scripts
```bash
chmod +x /home/vscode/workspace/experiments/yolo/*.sh
```

## Next Steps

After completing Phase 1 and Phase 3:

1. **Implement Phase 2** (Quality Analysis):
   - Create `analyze_quality.py` to analyze layer-wise detection quality
   - Determine optimal cancellation points
   - Generate quality progression plots

2. **Implement Phase 4** (Full Sweep):
   - Create configurations for various batch sizes
   - Test reactive vs proactive modes
   - Test single vs multi-threaded executors
   - Create `run_full_sweep.sh` script

3. **Write Paper**:
   - Use generated plots and metrics
   - Compare baseline vs maximum throughput
   - Analyze quality-performance tradeoffs
   - Discuss optimal cancellation strategies

## Expected Results

### Phase 1 (Baseline)
- Layer-by-layer processing times
- Detection quality progression across layers
- Identify at which layer detection quality plateaus

### Phase 3 (Maximum Throughput)
- Maximum achievable throughput
- Comparison with baseline runtime
- Understanding of batch processing benefits

## Contact

For questions or issues, please refer to the main project README or the experimental plan document.
