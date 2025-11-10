# YOLO Experiments - Quick Workflow Reference

## Complete Workflow (Run in Order)

### 0️⃣ Generate Baseline Configs (First Time Only)
```bash
python3 0_generate_baseline_configs.py
```
Creates configuration files for Steps 1 and 3.  
**Output:** `configs/phase1_*.yaml` and `configs/phase3_*.yaml`

### 0️⃣ Test Setup (Optional)
```bash
./0_test_setup.sh
```
Quick test to verify environment is working.

---

## SYSTEM EVALUATION

### 1️⃣ Collect Baseline Data
```bash
./1_collect_baseline.sh
```
**Duration:** ~30-60 minutes (3 trials)  
**Output:** `traces/phase1_baseline_trial{1,2,3}/`

### 2️⃣a Analyze Detection Quality
```bash
python3 2a_analyze_quality.py
```
**Output:** `results/quality_analysis/` (plots + CSV)  
**Shows:** Layer-wise detection progression, optimal cancellation points

### 2️⃣b Analyze Block Sizes
```bash
python3 2b_analyze_blocks.py
```
**Output:** `results/block_analysis/` (plots + CSV)  
**Shows:** Total delay for different block sizes

### 3️⃣ Measure Throughput
```bash
./3_measure_throughput.sh
```
**Duration:** ~20-40 minutes (4 configs × 3 trials)  
**Output:** `traces/phase3_{sync|async}_{single|multi}_trial{1,2,3}/`

### 4️⃣ Analyze Throughput
```bash
python3 4_analyze_throughput.py
```
**Output:** `results/runtime_analysis/` (plots + CSV)  
**Shows:** Throughput comparison, layer/exit computation times

**⚠️ REVIEW OUTPUTS from Steps 2a, 2b, and 4 before proceeding!**

---

## CANCELLATION EXPERIMENTS

### 5️⃣ Generate Configs
```bash
python3 5_generate_configs.py
```
**Output:** `configs/phase4_*.yaml` (16 server + 1 client)

### 6️⃣ Run Cancellation Experiments
```bash
./6_run_experiments.sh
```
**Duration:** ~1.5-3 hours (16 configs × 3 trials = 48 experiments)  
**Output:** `traces/phase4_bs{1,8,16,25}_proactive_{sync|async}_{single|multi}_trial{1,2,3}/`

**To test single config:**
```bash
./6a_test_single_config.sh bs1_proactive_sync_single
```

### 7️⃣ Analyze Cancellation Performance
```bash
python3 7_analyze_cancellation.py
```
**Output:** `results/phase4_analysis/` (plots + CSV + JSON)  
**Shows:** Cancellation delay, total runtime, layers processed

---

## Quick Commands

### Check what exists
```bash
ls -la traces/        # View collected traces
ls -la results/       # View analysis outputs
ls -la configs/       # View configuration files
```

### Clean up
```bash
lttng destroy --all   # Clean up LTTng sessions
rm -rf traces/*       # Remove all traces
rm -rf results/*      # Remove all results
```

### Re-run analysis only
```bash
# System evaluation
python3 2a_analyze_quality.py
python3 2b_analyze_blocks.py
python3 4_analyze_throughput.py

# Cancellation analysis
python3 7_analyze_cancellation.py
```

---

## File Organization

```
experiments/yolo/
├── 0_generate_baseline_configs.py  # Generate baseline/throughput configs (run first!)
├── 0_test_setup.sh                 # Optional environment test
├── 1_collect_baseline.sh           # Collect baseline traces
├── 2a_analyze_quality.py           # Quality analysis
├── 2b_analyze_blocks.py            # Block size analysis
├── 3_measure_throughput.sh         # Throughput experiments
├── 4_analyze_throughput.py         # Throughput analysis
├── 5_generate_configs.py           # Generate cancellation configs
├── 6_run_experiments.sh            # Run cancellation experiments
├── 6a_test_single_config.sh        # Test single config
├── 7_analyze_cancellation.py       # Cancellation analysis
├── README.md                       # Full documentation
├── WORKFLOW.md                     # This quick reference
├── configs/                        # YAML configurations
├── traces/                         # LTTng trace outputs
└── results/                        # Analysis outputs
```

---

## Expected Timeline

- **Step 1:** 30-60 minutes
- **Step 2a-2b:** 5-10 minutes
- **Step 3:** 20-40 minutes
- **Step 4:** 5-10 minutes
- **Review:** 30-60 minutes (manual)
- **Step 5:** < 1 minute
- **Step 6:** 1.5-3 hours
- **Step 7:** 10-15 minutes

**Total:** ~3-5 hours of automated experiments + manual review time
