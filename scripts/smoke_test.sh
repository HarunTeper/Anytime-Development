#!/bin/bash
# Smoke test — validates build, unit tests, and tracing infrastructure
# Usage: ./scripts/smoke_test.sh
# Expected duration: <2 minutes

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
PACKAGES_DIR="${WORKSPACE_DIR}/packages"
MC_EXPERIMENT_DIR="${WORKSPACE_DIR}/experiments/monte_carlo"

echo "========================================="
echo "Anytime ROS 2 — Smoke Test"
echo "========================================="
echo ""
echo "Workspace: ${WORKSPACE_DIR}"
echo ""

passed=0
failed=0

pass_phase() {
    echo "  PASSED"
    passed=$((passed + 1))
}

fail_phase() {
    echo "  FAILED: $1"
    failed=$((failed + 1))
}

# ─────────────────────────────────────────────
# Phase 1: Build workspace
# ─────────────────────────────────────────────
echo "-----------------------------------------"
echo "Phase 1/5: Building workspace"
echo "-----------------------------------------"

cd "${PACKAGES_DIR}"
if colcon build --symlink-install 2>&1; then
    pass_phase
else
    fail_phase "colcon build failed"
fi

source install/setup.bash

# ─────────────────────────────────────────────
# Phase 2: Run unit tests
# ─────────────────────────────────────────────
echo ""
echo "-----------------------------------------"
echo "Phase 2/5: Running unit tests"
echo "-----------------------------------------"

if colcon test --packages-select anytime_core anytime_monte_carlo 2>&1; then
    # Check results
    if colcon test-result --verbose 2>&1; then
        pass_phase
    else
        fail_phase "some unit tests failed"
    fi
else
    fail_phase "colcon test failed"
fi

# ─────────────────────────────────────────────
# Phase 3: Generate Monte Carlo configs
# ─────────────────────────────────────────────
echo ""
echo "-----------------------------------------"
echo "Phase 3/5: Generating Monte Carlo configs"
echo "-----------------------------------------"

cd "${MC_EXPERIMENT_DIR}"
if python3 generate_configs.py 2>&1; then
    pass_phase
else
    fail_phase "config generation failed"
fi

# ─────────────────────────────────────────────
# Phase 4: Run 5-second Monte Carlo experiment
# ─────────────────────────────────────────────
echo ""
echo "-----------------------------------------"
echo "Phase 4/5: Running 5-second Monte Carlo experiment"
echo "-----------------------------------------"

CONFIG_DIR="${MC_EXPERIMENT_DIR}/configs"
TRACE_DIR="${MC_EXPERIMENT_DIR}/traces"
TEST_CONFIG="batch_1024_reactive_single"
RUN_DURATION=5

test_trace="${TRACE_DIR}/smoke_test"
rm -rf "${test_trace}"
mkdir -p "${test_trace}"

# Source workspace again (we changed directory)
cd "${PACKAGES_DIR}"
source install/setup.bash

# Setup LTTng
lttng destroy smoke_test 2>/dev/null || true
lttng create smoke_test --output="${test_trace}"
lttng enable-event --userspace 'anytime:*'
lttng add-context --userspace --type=vpid
lttng add-context --userspace --type=vtid
lttng add-context --userspace --type=procname
lttng start

# Launch experiment
server_config="${CONFIG_DIR}/${TEST_CONFIG}_server.yaml"
client_config="${CONFIG_DIR}/${TEST_CONFIG}_client.yaml"

ros2 launch experiments monte_carlo.launch.py \
    server_config:="${server_config}" \
    client_config:="${client_config}" \
    use_multi_threaded:=false \
    log_level:=info \
    > /dev/null 2>&1 &

LAUNCH_PID=$!

# Wait for experiment duration
sleep ${RUN_DURATION}

# Cleanup
kill ${LAUNCH_PID} 2>/dev/null || true
sleep 1
kill -9 ${LAUNCH_PID} 2>/dev/null || true
pkill -9 -f 'anytime_monte_carlo' 2>/dev/null || true
pkill -9 -f 'ros2' 2>/dev/null || true
sleep 2

# Stop tracing
lttng stop
lttng destroy smoke_test

if [ -d "${test_trace}" ] && [ "$(ls -A ${test_trace})" ]; then
    pass_phase
else
    fail_phase "no trace data generated"
fi

# ─────────────────────────────────────────────
# Phase 5: Verify traces
# ─────────────────────────────────────────────
echo ""
echo "-----------------------------------------"
echo "Phase 5/5: Verifying traces"
echo "-----------------------------------------"

event_count=$(babeltrace "${test_trace}" | grep -c "anytime:" || echo "0")
echo "  Events captured: ${event_count}"

if [ "${event_count}" -gt 0 ]; then
    echo "  Sample events:"
    babeltrace "${test_trace}" | grep "anytime:" | head -5
    pass_phase
else
    fail_phase "no anytime events found in trace"
fi

# ─────────────────────────────────────────────
# Summary
# ─────────────────────────────────────────────
echo ""
echo "========================================="
echo "Smoke Test Summary"
echo "========================================="
echo "  Passed: ${passed}/5"
echo "  Failed: ${failed}/5"
echo ""

if [ ${failed} -eq 0 ]; then
    echo "ALL PHASES PASSED"
    echo ""
    echo "Next steps:"
    echo "  - Run quick experiments:  ./experiments/monte_carlo/run_quick.sh"
    echo "  - Run full experiments:   ./experiments/monte_carlo/run_monte_carlo_experiments.sh"
    exit 0
else
    echo "SOME PHASES FAILED"
    exit 1
fi
