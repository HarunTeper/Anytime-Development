# Step 6: Bash Wrapper Scripts

**Status:** Completed
**Day:** 2 (Wed Feb 25)
**Priority:** HIGH
**Time:** ~1.5h
**Dependencies:** Step 1

## Why

Evaluators need a simple entry point. Replaces the cut Python CLI.

**AE Impact:** Criterion 2 (Instructions) - "single command or clearly defined short series of steps."

## New Files

### `scripts/run_all.sh`

```bash
#!/bin/bash
# End-to-end artifact evaluation runner
# Usage: ./scripts/run_all.sh [--quick|--full] [--cpu-only]
```

Steps:

1. Source ROS2 + workspace
2. Run smoke test
3. Run Monte Carlo experiments (quick or full)
4. Run Interference experiments (quick or full)
5. Run YOLO experiments (if GPU available, skip with `--cpu-only`)
6. Generate all analysis plots
7. Print summary with paths to results

### `scripts/reproduce_figure.sh`

```bash
#!/bin/bash
# Reproduce a specific paper figure
# Usage: ./scripts/reproduce_figure.sh <figure_number>
# Examples:
#   ./scripts/reproduce_figure.sh 5a    # Monte Carlo total iterations
#   ./scripts/reproduce_figure.sh 5b    # Monte Carlo cancel-to-finish latency
#   ./scripts/reproduce_figure.sh 6     # Interference (Fig 6a+6b+Table I)
#   ./scripts/reproduce_figure.sh 7a    # YOLO quality progression
#   ./scripts/reproduce_figure.sh 7b    # YOLO runtime comparison
```

Maps each figure to the specific experiment + analysis script commands.

## Verification

```bash
./scripts/run_all.sh --quick --cpu-only
./scripts/reproduce_figure.sh 5a
```
