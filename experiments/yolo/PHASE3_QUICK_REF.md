# Phase 3 Quick Reference

## Run the Experiment
```bash
cd /home/vscode/workspace/experiments/yolo
./run_phase3_max_throughput.sh
```

## What It Tests
- **4 configurations** × **3 trials** = **12 runs total**
- Sync+Single, Sync+Multi, Async+Single, Async+Multi
- All with batch_size=25, no cancellation (full processing)

## Output Locations

### Traces
```
traces/phase3_sync_single_trial{1,2,3}/
traces/phase3_sync_multi_trial{1,2,3}/
traces/phase3_async_single_trial{1,2,3}/
traces/phase3_async_multi_trial{1,2,3}/
```

### Timing Summary
```
results/phase3_timing_summary.txt
```

## View Results
```bash
# Timing summary
cat results/phase3_timing_summary.txt

# Detailed trace analysis
python3 evaluate_yolo.py
```

## Configuration Files
- **Server**: `configs/phase3_server_{sync,async}_{single,multi}.yaml`
- **Client**: `configs/phase3_client.yaml`

## Expected Duration
- ~45-60 seconds per trial
- 3 trials × 4 configs = 12 trials
- Total: ~10-15 minutes (including setup/cleanup between runs)

## Analysis Questions
1. Which config is fastest?
2. Multi-threading speedup?
3. Async speedup?
4. Best combination?
