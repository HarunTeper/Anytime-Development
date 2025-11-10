# YOLO Experiments

Evaluate Anytime YOLO performance with different cancellation strategies and configurations.

## Complete Workflow (In Order)

### Step 0a: Generate Baseline Configs (First Time Only)
```bash
python3 0_generate_baseline_configs.py
```
Creates configuration files for Steps 1 and 3 (baseline and throughput).
- **Output:** `configs/phase1_*.yaml` and `configs/phase3_*.yaml`

### Step 0b: Test Setup (Optional)
```bash
./0_test_setup.sh
```
Quick test to verify your environment is working correctly.

---

### **SYSTEM EVALUATION**

### Step 1: Collect Baseline Data
```bash
./1_collect_baseline.sh
```
- Runs 3 trials with batch_size=1, single-threaded
- Collects layer-wise detection quality and timing data
- **Output:** `traces/phase1_baseline_trial{1,2,3}/`

### Step 2a: Analyze Detection Quality
```bash
python3 2a_analyze_quality.py
```
- Analyzes layer-wise detection progression
- Determines optimal cancellation points
- **Output:** `results/quality_analysis/` (plots + CSV)

### Step 2b: Analyze Block Sizes
```bash
python3 2b_analyze_blocks.py
```
- Evaluates total delay for different block sizes
- Shows cumulative computation costs
- **Output:** `results/block_analysis/` (plots + CSV)

### Step 3: Measure Maximum Throughput
```bash
./3_measure_throughput.sh
```
- Tests 4 configurations: sync/async × single/multi-threaded
- All use batch_size=25 (full model, no cancellation)
- **Output:** `traces/phase3_{sync|async}_{single|multi}_trial{1,2,3}/`

### Step 4: Analyze Throughput Results
```bash
python3 4_analyze_throughput.py
```
- Compares throughput across configurations
- Analyzes layer computation and exit calculation times
- **Output:** `results/runtime_analysis/` (plots + CSV)

**→ Review outputs from Steps 2a, 2b, and 4 before proceeding**

---

### **CANCELLATION EXPERIMENTS**

### Step 5: Generate Cancellation Configurations
```bash
python3 5_generate_configs.py
```
- Creates configs for 16 combinations:
  - Block sizes: 1, 8, 16, 25
  - Mode: proactive
  - Sync: sync, async
  - Threading: single, multi
- **Output:** `configs/phase4_*.yaml`

### Step 6: Run Cancellation Experiments
```bash
./6_run_experiments.sh
```
- Runs all 16 configurations × 3 trials = 48 experiments
- Client cancels after 16 layers OR score threshold 0.8
- **Output:** `traces/phase4_bs{1,8,16,25}_proactive_{sync|async}_{single|multi}_trial{1,2,3}/`
- **Note:** To test a single config: `./6a_test_single_config.sh bs1_proactive_sync_single`

### Step 7: Analyze Cancellation Performance
```bash
python3 7_analyze_cancellation.py
```
- Measures cancellation delay and total runtime
- Compares performance across all configurations
- **Output:** `results/phase4_analysis/` (plots + CSV)
---

## Prerequisites

1. **Build ROS2 workspace**:
   ```bash
   cd /home/vscode/workspace/packages
   colcon build --symlink-install
   ```

2. **Download YOLO weights**:
   ```bash
   cd /home/vscode/workspace/packages/src/anytime_yolo
   wget https://tu-dortmund.sciebo.de/s/W86QE9hUscsUPeM/download -O weights.zip
   unzip -o weights.zip -d .
   rm weights.zip
   ```

3. **Download test images**:
   ```bash
   mkdir -p /home/vscode/workspace/packages/src/video_publisher/images
   cd /home/vscode/workspace/packages/src/video_publisher/images
   wget https://tu-dortmund.sciebo.de/s/aA9MDhgN2lBmeZk/download -O images.zip
   unzip -o images.zip -d .
   rm images.zip
   ```

4. **Install Python dependencies**:
   ```bash
   pip3 install pandas numpy matplotlib pyyaml
   ```

---

## What Each Script Does

### Baseline Collection (Step 1)
- **Purpose:** Establish baseline performance and quality data
- **Configuration:** Single-threaded, batch_size=1, all 25 layers
- **Duration:** ~30-60 minutes for 3 trials
- **Use cases:** Understanding layer-wise quality progression, identifying when detections stabilize

### Quality Analysis (Step 2a)
- **Purpose:** Determine optimal cancellation points
- **Outputs:**
  - Detection progression curves (how many detections at each layer)
  - Quality ratio plots (percentage of final detections reached)
  - Layer computation times
  - Recommendations for cancellation thresholds

### Block Analysis (Step 2b)
- **Purpose:** Understand cost of different block sizes
- **Outputs:**
  - Total delay vs block size
  - Number of blocks vs block size
  - Per-block delay distributions
  - Helps choose block sizes for Step 6

### Throughput Measurement (Step 3)
- **Purpose:** Find maximum throughput configuration
- **Configurations tested:**
  - sync + single-threaded
  - sync + multi-threaded
  - async + single-threaded
  - async + multi-threaded
- **Duration:** ~20-40 minutes for 4 configs × 3 trials

### Throughput Analysis (Step 4)
- **Purpose:** Compare performance of different configurations
- **Outputs:**
  - Total goal time comparisons
  - Throughput (images/second)
  - Layer and exit computation time breakdowns
  - Cumulative runtime plots

### Config Generation (Step 5)
- **Purpose:** Auto-generate all configuration files for cancellation experiments
- **Output:** 16 YAML files + 1 client config

### Cancellation Experiments (Step 6)
- **Purpose:** Measure cancellation responsiveness
- **Client behavior:** Cancel after 16 layers OR score ≥ 0.8 (whichever comes first)
- **Duration:** ~1.5-3 hours for 16 configs × 3 trials
- **Output:** Traces with cancellation timing data

### Cancellation Analysis (Step 7)
- **Purpose:** Evaluate cancellation performance
- **Outputs:**
  - Cancellation delay (time from cancel request to result)
  - Total runtime comparison
  - Layers processed before cancellation
  - Recommendations for best configuration

---

## Technical Details

### Configuration Files (YAML)
Located in `configs/`, these specify:
- **batch_size**: Number of layers processed per block (1, 8, or 25)
- **is_reactive_proactive**: Mode ("reactive" or "proactive")
- **is_sync_async**: Executor type ("sync" or "async")
- **multi_threading**: Threading mode (true/false)
- **cancel_after_layers**: Client cancellation threshold (16 for Step 6)
- **score_threshold**: Quality-based cancellation (0.8 for Step 6)

### Metrics Collected
- **Total runtime**: Goal start to result received
- **Cancellation delay**: Cancel request to result received
- **Layer computation time**: Per-layer processing duration
- **Exit calculation time**: Time to compute detections after each block
- **Detection counts**: Number of objects detected at each exit
- **Throughput**: Images processed per second

### LTTng Tracepoints Used
- `anytime:yolo_layer_start` / `yolo_layer_end` - Layer computation timing
- `anytime:yolo_exit_calculation_start` / `yolo_exit_calculation_end` - Exit timing + detection count
- `anytime:anytime_base_activate` - Goal start
- `anytime:anytime_base_reset` - Goal reset
- Additional tracepoints for initialization and results

---

## Troubleshooting

### LTTng Session Errors
```bash
# Destroy any existing sessions
lttng destroy yolo_phase1_baseline
lttng destroy verify_test
lttng list  # Check active sessions
```

### Missing Images or Weights
- **Images:** `/home/vscode/workspace/packages/src/video_publisher/images/`
- **Weights:** `/home/vscode/workspace/packages/src/anytime_yolo/weights_32/`
- Re-run download tasks or use manual wget commands above

### Script Permission Errors
```bash
chmod +x /home/vscode/workspace/experiments/yolo/*.sh
```

### Python Import Errors
```bash
pip3 install --upgrade pandas numpy matplotlib pyyaml
```

### Trace Parsing Errors
- Check babeltrace is installed: `babeltrace2 --version`
- Verify trace directory contains data: `ls -la traces/*/`

---

## Expected Outcomes

After completing all steps, you will have:

1. **System characterization:**
   - Baseline quality progression (when detections stabilize)
   - Block size impact analysis (optimal block sizes)
   - Maximum throughput measurements (best configuration)

2. **Cancellation performance data:**
   - Cancellation delay across 24 configurations
   - Total runtime comparison
   - Quality-performance tradeoffs

3. **Visualizations:**
   - Quality progression plots
   - Throughput comparison charts
   - Cancellation delay heatmaps
   - Block size impact graphs

4. **Data exports:**
   - CSV summaries for paper tables
   - JSON files with detailed metrics
   - Recommended configurations for production use

---

## File Organization

```
experiments/yolo/
├── 0_generate_baseline_configs.py  # Generate baseline/throughput configs (run first!)
├── 0_test_setup.sh                 # Optional environment test
├── 1_collect_baseline.sh           # Step 1: Collect baseline traces
├── 2a_analyze_quality.py           # Step 2a: Quality analysis
├── 2b_analyze_blocks.py            # Step 2b: Block size analysis
├── 3_measure_throughput.sh         # Step 3: Throughput experiments
├── 4_analyze_throughput.py         # Step 4: Throughput analysis
├── 5_generate_configs.py           # Step 5: Generate cancellation configs
├── 6_run_experiments.sh            # Step 6: Run cancellation experiments
├── 6a_test_single_config.sh        # Step 6: Test one config
├── 7_analyze_cancellation.py       # Step 7: Cancellation analysis
├── configs/                        # Generated YAML configurations
├── traces/                         # LTTng trace outputs
└── results/                        # Analysis outputs (plots, CSV, JSON)
    ├── quality_analysis/
    ├── block_analysis/
    ├── runtime_analysis/
    └── phase4_analysis/
```
