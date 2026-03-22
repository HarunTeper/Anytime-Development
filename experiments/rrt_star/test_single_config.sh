#!/bin/bash
# Test a single RRT* configuration for sanity checking
# Usage: ./test_single_config.sh [block_size] [mode] [threading] [map]

set -e

BLOCK_SIZE="${1:-256}"
MODE="${2:-reactive}"
THREADING="${3:-single}"
MAP="${4:-depot}"
DURATION="${5:-10}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
CONFIG_DIR="${SCRIPT_DIR}/configs"
TRACE_DIR="${SCRIPT_DIR}/traces"
PACKAGES_DIR="${WORKSPACE_DIR}/packages"

config_name="block_${BLOCK_SIZE}_${MODE}_${THREADING}_${MAP}"

echo "========================================="
echo "Single Config Test: ${config_name}"
echo "Duration: ${DURATION}s"
echo "========================================="

# Cleanup on interrupt
cleanup() {
    echo "Cleaning up..."
    lttng stop 2>/dev/null || true
    pkill -9 -f 'component_container' 2>/dev/null || true
    pkill -9 -f 'ros2' 2>/dev/null || true
    sleep 1
    lttng destroy rrt_star_test 2>/dev/null || true
}
trap cleanup INT TERM

pkill lttng-sessiond 2>/dev/null || true
sleep 2
lttng-sessiond --daemonize 2>/dev/null || true
sleep 1

cd "${PACKAGES_DIR}"
source install/setup.bash

MAPS_DIR="$(ros2 pkg prefix anytime_rrt_star)/share/anytime_rrt_star/maps"

if [ -d "${CONFIG_DIR}" ]; then
    find "${CONFIG_DIR}" -name "*_server.yaml" -exec sed -i "s|MAPS_DIR|${MAPS_DIR}|g" {} \;
fi

server_config="${CONFIG_DIR}/${config_name}_server.yaml"
client_config="${CONFIG_DIR}/${config_name}_client.yaml"

if [ ! -f "${server_config}" ]; then
    echo "ERROR: Config not found: ${server_config}"
    echo "Run generate_configs.py first."
    exit 1
fi

trace_output="${TRACE_DIR}/test_${config_name}"
rm -rf "${trace_output}"
mkdir -p "${trace_output}"

lttng destroy rrt_star_test 2>/dev/null || true
lttng create rrt_star_test --output="${trace_output}"

lttng enable-event --userspace 'anytime:*'
lttng add-context --userspace --type=vpid
lttng add-context --userspace --type=vtid
lttng add-context --userspace --type=procname
lttng start

if [ "${THREADING}" = "multi" ]; then
    use_multi_threaded="true"
else
    use_multi_threaded="false"
fi

ros2 launch experiments rrt_star.launch.py \
    server_config:="${server_config}" \
    client_config:="${client_config}" \
    use_multi_threaded:=${use_multi_threaded} \
    log_level:=info &

LAUNCH_PID=$!
sleep ${DURATION}

lttng stop
sleep 1
kill ${LAUNCH_PID} 2>/dev/null || true
sleep 1
kill -9 ${LAUNCH_PID} 2>/dev/null || true
pkill -9 -f 'component_container' 2>/dev/null || true
pkill -9 -f 'ros2' 2>/dev/null || true
sleep 1
lttng destroy rrt_star_test

echo ""
echo "Trace saved to: ${trace_output}"
echo "View with: babeltrace ${trace_output}"
