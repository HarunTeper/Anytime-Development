# Step 5: Smoke Test + Simplified Experiments

**Status:** Completed
**Day:** 2 (Wed Feb 25)
**Priority:** HIGH
**Time:** ~1.5h
**Dependencies:** Steps 1, 4

## Why

AE webpage says "authors may prepare simplified experiments with shorter running times that demonstrate the same trends."

**AE Impact:** Criterion 2 (Instructions) - having quick-run options makes evaluation much easier.

## New Files

### `scripts/smoke_test.sh`

Fast validation script (<2 minutes):

- Build workspace
- Run unit tests
- Run a 5-second Monte Carlo experiment
- Verify traces collected
- Print PASS/FAIL summary

### `experiments/monte_carlo/run_quick.sh`

Shortened version of `run_monte_carlo_experiments.sh`:

- 3 batch sizes instead of 7 (1024, 16384, 65536)
- 1 trial instead of 3
- 5-second runs instead of 10
- Total: ~5 minutes (demonstrates same trends as full run)

### `experiments/interference/run_quick.sh`

Same pattern:

- 3 batch sizes, 1 trial, 5-second runs
- Total: ~3 minutes

## Verification

```bash
./scripts/smoke_test.sh
./experiments/monte_carlo/run_quick.sh
./experiments/interference/run_quick.sh
```
