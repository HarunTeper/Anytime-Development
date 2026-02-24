#!/bin/bash
# Quick test script - runs a single configuration to verify setup

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
EXPERIMENT_DIR="${SCRIPT_DIR}"
CONFIG_DIR="${EXPERIMENT_DIR}/configs"
TRACE_DIR="${EXPERIMENT_DIR}/traces"
PACKAGES_DIR="${WORKSPACE_DIR}/packages"

# Test configuration
TEST_CONFIG="batch_1_reactive_multi"
RUN_DURATION=10

echo "========================================="
echo "Monte Carlo Single Configuration Test"
echo "========================================="
echo ""
echo "Testing configuration: ${TEST_CONFIG}"
echo "Run duration: ${RUN_DURATION}s"
echo ""

# Verify configs exist
if [ ! -f "${CONFIG_DIR}/${TEST_CONFIG}_server.yaml" ]; then
    echo "Error: Configuration files not found!"
    echo "Please run: python3 generate_configs.py"
    exit 1
fi

# Source workspace
cd "${PACKAGES_DIR}"
source install/setup.bash

# Create test trace directory
test_trace="${TRACE_DIR}/test_${TEST_CONFIG}"
rm -rf "${test_trace}"
mkdir -p "${test_trace}"

echo "[1/5] Cleaning up any existing LTTng sessions..."
lttng destroy test_monte_carlo 2>/dev/null || true

echo "[2/5] Creating LTTng session..."
lttng create test_monte_carlo --output="${test_trace}"

echo "[3/5] Enabling tracepoints..."
lttng enable-event --userspace 'anytime:*'
lttng add-context --userspace --type=vpid
lttng add-context --userspace --type=vtid

echo "[4/5] Starting trace and launching experiment..."
lttng start

# Launch experiment
server_config="${CONFIG_DIR}/${TEST_CONFIG}_server.yaml"
client_config="${CONFIG_DIR}/${TEST_CONFIG}_client.yaml"

timeout ${RUN_DURATION} ros2 launch experiments monte_carlo.launch.py \
    server_config:="${server_config}" \
    client_config:="${client_config}" \
    use_multi_threaded:=true \
    log_level:=info &

LAUNCH_PID=$!

# Wait for completion
wait ${LAUNCH_PID} 2>/dev/null || true

# Kill any remaining processes
kill ${LAUNCH_PID} 2>/dev/null || true
sleep 1
kill -9 ${LAUNCH_PID} 2>/dev/null || true

# Kill any remaining Monte Carlo processes
pkill -9 -f 'component_container' 2>/dev/null || true
pkill -9 -f 'anytime_monte_carlo' 2>/dev/null || true
pkill -9 -f 'ros2' 2>/dev/null || true

sleep 2

echo "[5/5] Stopping trace..."
lttng stop
lttng destroy test_monte_carlo

# Verify trace
echo ""
echo "========================================="
echo "Trace Verification"
echo "========================================="
echo ""

if [ -d "${test_trace}" ] && [ "$(ls -A ${test_trace})" ]; then
    echo "✓ Trace directory created: ${test_trace}"
    
    # Count events
    event_count=$(babeltrace "${test_trace}" | grep -c "anytime:" || echo "0")
    echo "✓ Events captured: ${event_count}"
    
    if [ ${event_count} -gt 0 ]; then
        echo ""
        echo "Sample events:"
        babeltrace "${test_trace}" | grep "anytime:" | head -10
        
        echo ""
        echo "========================================="
        echo "Test PASSED! ✓"
        echo "========================================="
        echo ""
        echo "You can now run the full experiment suite:"
        echo "  ./run_monte_carlo_experiments.sh"
        echo ""
    else
        echo ""
        echo "Warning: No anytime events found in trace"
        echo "This may indicate tracing is not working properly"
        exit 1
    fi
else
    echo "✗ Error: No trace data generated"
    exit 1
fi
