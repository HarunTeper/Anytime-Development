# Step 10: Artifact Evaluation Guide

**Status:** Completed
**Day:** 3 (Thu Feb 26)
**Priority:** HIGH
**Time:** ~1.5h
**Dependencies:** Steps 6, 9

## Why

This is the first document evaluators read. It must map paper claims to concrete commands.

**AE Impact:** All three criteria. This is the keystone document.

## New File: `ARTIFACT_EVALUATION.md` (repository root)

### Structure

1. **Paper:** Title, authors, venue
2. **Hardware Requirements:** x86_64 Linux, RAM, disk, Docker; GPU optional (CUDA 12.5)
3. **Quick Start** (< 10 min):

   ```bash
   docker compose build anytime-cpu
   docker compose run --rm anytime-cpu bash
   # Inside container:
   cd packages && colcon build --symlink-install && source install/setup.bash
   ./scripts/smoke_test.sh
   ```

4. **Reproducing Paper Figures** - explicit mapping:

   | Paper Element | Command | Duration | Hardware |
   | ------------- | ------- | -------- | -------- |
   | Figure 5a (segment count) | `./scripts/reproduce_figure.sh 5a` | ~40 min | CPU |
   | Figure 5b (cancel delay) | `./scripts/reproduce_figure.sh 5b` | ~40 min | CPU |
   | Figure 6a+6b + Table I | `./scripts/reproduce_figure.sh 6` | ~40 min | CPU |
   | Figure 7a (quality progression) | `./scripts/reproduce_figure.sh 7a` | ~30 min | GPU |
   | Figure 7b (runtime comparison) | `./scripts/reproduce_figure.sh 7b` | ~3 hrs | GPU |

5. **Quick Reproduction** (~15 min, CPU only):

   ```bash
   ./scripts/run_all.sh --quick --cpu-only
   ```

6. **Full Reproduction** (~5 hrs with GPU):

   ```bash
   ./scripts/run_all.sh --full
   ```

7. **Output Locations** - where to find results, CSV data, and plots
8. **Environment Support** - Docker tested on Ubuntu 22.04/24.04
9. **Troubleshooting** - common issues and fixes
10. **Project Structure** - brief overview pointing to docs/

### Paper PDF

AE webpage requires: "be sure to include a version of the accepted paper related to the artifact." `main.pdf` already exists at repo root - verify it's the latest version and mention it in the guide.
