# analyze_runtime.py - Quick Reference

## Purpose
Dedicated script for analyzing Phase 3 computation times across different configurations (sync/async, single/multi-threaded).

## Usage
```bash
cd /home/vscode/workspace/experiments/yolo
python3 analyze_runtime.py
```

## What It Analyzes
- **Total goal processing time** - Complete time per image
- **Layer computation times** - Time to process each layer
- **Exit calculation times** - Time to calculate results per layer
- **Complete computation time** - Stacked visualization (layer + exit)
- **Throughput comparison** - Images per second across configurations

## Input
Analyzes all Phase 3 traces in `traces/phase3_*`

## Output Directory
`results/runtime_analysis/`

## Generated Files

### Plots
1. **total_goal_time_comparison.png** - Bar chart comparing average goal processing time
2. **throughput_comparison.png** - Bar chart showing throughput (imgs/sec)
3. **layer_computation_by_config.png** - Line plots of layer computation times per config
4. **exit_calculation_by_config.png** - Line plots of exit calculation times per config
5. **stacked_layer_times_by_config.png** - Stacked bar charts (one subplot per config)
6. **combined_stacked_comparison.png** - Side-by-side comparison of all configs

### Data Files
- **runtime_analysis.json** - Complete metrics in JSON format
- **runtime_summary.txt** - Human-readable summary with statistics

## Key Metrics Per Configuration

### Per Goal
- Average processing time (ms)
- Standard deviation
- Min/Max times
- Throughput (images/sec)

### Per Layer
- Computation time (mean ± std)
- Exit calculation time (mean ± std)
- Total time (computation + exit)

## Configuration Naming
- `sync_single` - Synchronous + Single-threaded
- `sync_multi` - Synchronous + Multi-threaded
- `async_single` - Asynchronous + Single-threaded
- `async_multi` - Asynchronous + Multi-threaded

## Differences from evaluate_yolo.py
- **Focus**: Runtime/timing analysis only (not detection quality)
- **Target**: Phase 3 experiments only (not general-purpose)
- **Detail**: Layer-by-layer breakdown with stacked visualization
- **Comparison**: Direct comparison across configurations
- **Output**: Dedicated runtime_analysis directory
