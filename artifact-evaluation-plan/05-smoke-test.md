# 05 — Smoke Test Script

**Day:** 2 (Wed Feb 25)
**Priority:** HIGH
**Estimated time:** 1 hour
**Depends on:** 03 (unit tests), 04 (integration tests)

---

## Overview

A fast validation script (<2 minutes) that verifies the entire toolchain works. Used as the first step in the automated setup and as a quick sanity check during development.

---

## New File: `scripts/smoke_test.sh`

```bash
#!/bin/bash
set -euo pipefail

# Anytime ROS 2 — Smoke Test
# Validates build, tests, and basic experiment functionality
# Expected runtime: <2 minutes

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(dirname "$SCRIPT_DIR")}"
PACKAGES_DIR="${WORKSPACE_DIR}/packages"
CPU_ONLY="${CPU_ONLY:-false}"

PASS=0
FAIL=0

report() {
  local status=$1 name=$2
  if [ "$status" -eq 0 ]; then
    echo "[PASS] $name"
    ((PASS++))
  else
    echo "[FAIL] $name"
    ((FAIL++))
  fi
}

# Step 1: Build workspace
echo "=== Step 1: Building workspace ==="
cd "$PACKAGES_DIR"
colcon build --symlink-install 2>&1 | tail -5
report $? "Workspace build"

# Step 2: Source workspace
source "${PACKAGES_DIR}/install/setup.bash"

# Step 3: Run unit tests
echo "=== Step 2: Running unit tests ==="
colcon test --packages-select anytime_core anytime_monte_carlo 2>&1 | tail -5
report $? "Unit tests"

# Step 4: Run GPU tests (if available)
if [ "$CPU_ONLY" != "true" ]; then
  echo "=== Step 3: Running GPU tests ==="
  colcon test --packages-select anytime_yolo 2>&1 | tail -5
  report $? "GPU tests"
fi

# Step 5: Quick Monte Carlo experiment (5 seconds)
echo "=== Step 4: Quick Monte Carlo experiment ==="
SESSION_NAME="smoke_test_mc"
TRACE_DIR="/tmp/smoke_test_traces"
rm -rf "$TRACE_DIR"

lttng destroy "$SESSION_NAME" 2>/dev/null || true
lttng create "$SESSION_NAME" --output="$TRACE_DIR"
lttng enable-event --userspace 'anytime:anytime_compute_entry'
lttng start

ros2 launch experiments monte_carlo.launch.py \
  server_config:=default_server.yaml \
  client_config:=default_client.yaml \
  use_multi_threaded:=false &
LAUNCH_PID=$!

sleep 5
kill $LAUNCH_PID 2>/dev/null || true
sleep 1

lttng stop
lttng destroy "$SESSION_NAME"

# Verify traces were collected
TRACE_COUNT=$(babeltrace2 "$TRACE_DIR" 2>/dev/null | wc -l)
if [ "$TRACE_COUNT" -gt 0 ]; then
  report 0 "Monte Carlo trace collection ($TRACE_COUNT events)"
else
  report 1 "Monte Carlo trace collection (0 events)"
fi

# Cleanup
rm -rf "$TRACE_DIR"
pkill -f "ros2" 2>/dev/null || true

# Summary
echo ""
echo "=== Smoke Test Summary ==="
echo "Passed: $PASS"
echo "Failed: $FAIL"
echo ""

if [ "$FAIL" -gt 0 ]; then
  echo "SMOKE TEST FAILED"
  exit 1
else
  echo "SMOKE TEST PASSED"
  exit 0
fi
```

---

## Features

- **Fast:** Under 2 minutes total
- **CPU-only mode:** Set `CPU_ONLY=true` to skip GPU tests
- **Self-cleaning:** Removes temporary traces and kills background processes
- **Clear output:** PASS/FAIL per step with summary
- **Exit code:** Returns 0 on success, 1 on failure (for CI integration)

---

## Verification

```bash
# Run smoke test
chmod +x scripts/smoke_test.sh
./scripts/smoke_test.sh

# Run in CPU-only mode
CPU_ONLY=true ./scripts/smoke_test.sh
```
