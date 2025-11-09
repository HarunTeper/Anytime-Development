# Phase 3: Maximum Throughput Experiments - Updated

## Overview
Phase 3 tests full processing performance (all 25 layers, no cancellation) across different configuration modes to measure throughput limits and identify optimal settings.

## Test Configurations

The script tests **4 configurations** with **3 trials each** (12 total runs):

### 1. Sync + Single-threaded
- **Config file**: `phase3_server_sync_single.yaml`
- **is_sync_async**: `sync`
- **multi_threading**: `false`
- **Purpose**: Baseline synchronous, single-threaded performance

### 2. Sync + Multi-threaded
- **Config file**: `phase3_server_sync_multi.yaml`
- **is_sync_async**: `sync`
- **multi_threading**: `true`
- **Purpose**: Test multi-threading benefit in synchronous mode

### 3. Async + Single-threaded
- **Config file**: `phase3_server_async_single.yaml`
- **is_sync_async**: `async`
- **multi_threading**: `false`
- **Purpose**: Test asynchronous processing without multi-threading

### 4. Async + Multi-threaded
- **Config file**: `phase3_server_async_multi.yaml`
- **is_sync_async**: `async`
- **multi_threading**: `true`
- **Purpose**: Test combined async + multi-threading (potentially fastest)

## Common Settings (All Configs)
- **batch_size**: 25 (process all layers at once)
- **is_reactive_proactive**: `proactive`
- **cancel_after_layers**: 25 (no cancellation)
- **cancel_layer_score**: `false`
- **weights_path**: `/home/vscode/workspace/packages/src/anytime_yolo/weights_32`

## Running the Experiment

```bash
cd /home/vscode/workspace/experiments/yolo
./run_phase3_max_throughput.sh
```

## Output Files

### Traces
Each configuration gets 3 trace directories:
- `traces/phase3_sync_single_trial{1,2,3}/`
- `traces/phase3_sync_multi_trial{1,2,3}/`
- `traces/phase3_async_single_trial{1,2,3}/`
- `traces/phase3_async_multi_trial{1,2,3}/`

### Timing Summary
- **File**: `results/phase3_timing_summary.txt`
- **Content**: Total processing time for each trial and configuration
- **Statistics**: Average, Min, Max times per configuration

## Expected Metrics

For each configuration, the script measures:
1. **Total processing time** (wall-clock time from start to finish)
2. **Per-layer timing** (from traces, analyzed separately)
3. **Throughput** (images per second)

## Analysis Questions

1. Which configuration is fastest for full processing?
2. How much speedup does multi-threading provide?
3. How much speedup does async mode provide?
4. What is the interaction between async and multi-threading?
5. What are the tradeoffs between throughput and latency?

## Next Steps After Completion

1. Review timing summary:
   ```bash
   cat results/phase3_timing_summary.txt
   ```

2. Analyze traces for detailed metrics:
   ```bash
   python3 evaluate_yolo.py
   ```

3. Compare configurations to determine optimal settings for different use cases

## Configuration Files

- **Server configs**: `configs/phase3_server_{sync,async}_{single,multi}.yaml`
- **Client config**: `configs/phase3_client.yaml` (shared across all tests)
