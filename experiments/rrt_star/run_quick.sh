#!/bin/bash
# Quick RRT* experiment run for development/CI
# Uses a small subset of configurations with short run duration

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Override experiment parameters for quick run
export BATCH_SIZES=(1 256 4096)
export MODES=("reactive" "proactive")
export THREADING=("single")
export MAPS=("depot")
export NUM_RUNS=1
export RUN_DURATION=5

echo "========================================="
echo "Quick RRT* Experiment Run"
echo "========================================="
echo "  Batch sizes: ${BATCH_SIZES[*]}"
echo "  Modes: ${MODES[*]}"
echo "  Threading: ${THREADING[*]}"
echo "  Maps: ${MAPS[*]}"
echo "  Duration: ${RUN_DURATION}s per config"
echo "  Total configs: $((${#BATCH_SIZES[@]} * ${#MODES[@]} * ${#THREADING[@]} * ${#MAPS[@]}))"
echo "========================================="
echo ""

# Source and run the main script with overridden parameters
# We inline the logic here to use the overridden variables

# Cleanup on interrupt
cleanup() {
    echo ""
    echo "Interrupted — cleaning up..."
    lttng stop 2>/dev/null || true
    pkill -9 -f 'component_container' 2>/dev/null || true
    pkill -9 -f 'anytime_rrt_star' 2>/dev/null || true
    pkill -9 -f 'ros2' 2>/dev/null || true
    sleep 1
    lttng destroy rrt_star_exp 2>/dev/null || true
}
trap cleanup INT TERM

# Preflight checks
if ! command -v lttng &>/dev/null; then
    echo "ERROR: lttng not found. Install lttng-tools."
    exit 1
fi

pkill lttng-sessiond 2>/dev/null || true
sleep 2
lttng-sessiond --daemonize 2>/dev/null || true
sleep 1

WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
EXPERIMENT_DIR="${SCRIPT_DIR}"
CONFIG_DIR="${EXPERIMENT_DIR}/configs"
TRACE_DIR="${EXPERIMENT_DIR}/traces"
RESULTS_DIR="${EXPERIMENT_DIR}/results"
PACKAGES_DIR="${WORKSPACE_DIR}/packages"

cd "${PACKAGES_DIR}"
source install/setup.bash

MAPS_DIR="$(ros2 pkg prefix anytime_rrt_star)/share/anytime_rrt_star/maps"

# Regenerate configs to ensure fresh MAPS_DIR placeholders (idempotent)
echo "Regenerating configs..."
rm -rf "${CONFIG_DIR}"
cd "${EXPERIMENT_DIR}"
python3 generate_configs.py
cd "${PACKAGES_DIR}"

echo "Updating map paths in config files..."
find "${CONFIG_DIR}" -name "*_server.yaml" -exec sed -i "s|MAPS_DIR|${MAPS_DIR}|g" {} \;

# Clean old output from previous runs
echo "Cleaning old traces and results..."
rm -rf "${TRACE_DIR}"
rm -rf "${RESULTS_DIR}"

mkdir -p "${TRACE_DIR}"
mkdir -p "${RESULTS_DIR}"

total_configs=$((${#BATCH_SIZES[@]} * ${#MODES[@]} * ${#THREADING[@]} * ${#MAPS[@]} * NUM_RUNS))
current_config=0

for batch_size in "${BATCH_SIZES[@]}"; do
    for mode in "${MODES[@]}"; do
        for thread_mode in "${THREADING[@]}"; do
            for map_name in "${MAPS[@]}"; do
                for run in $(seq 1 ${NUM_RUNS}); do
                    current_config=$((current_config + 1))
                    config_name="batch_${batch_size}_${mode}_${thread_mode}_${map_name}"
                    run_name="${config_name}_run${run}"

                    echo ""
                    echo "Running ${current_config}/${total_configs}: ${run_name}"

                    trace_output="${TRACE_DIR}/${run_name}"
                    rm -rf "${trace_output}"
                    mkdir -p "${trace_output}"
                    lttng destroy rrt_star_exp 2>/dev/null || true
                    lttng create rrt_star_exp --output="${trace_output}"

                    lttng enable-event --userspace anytime:anytime_compute_entry
                    lttng enable-event --userspace anytime:anytime_compute_exit
                    lttng enable-event --userspace anytime:anytime_server_handle_cancel
                    lttng enable-event --userspace anytime:anytime_base_deactivate
                    lttng enable-event --userspace anytime:anytime_client_goal_sent
                    lttng enable-event --userspace anytime:anytime_client_cancel_sent
                    lttng enable-event --userspace anytime:anytime_client_goal_finished
                    lttng enable-event --userspace anytime:anytime_send_feedback_entry
                    lttng enable-event --userspace anytime:anytime_send_feedback_exit
                    lttng enable-event --userspace anytime:anytime_calculate_result_entry
                    lttng enable-event --userspace anytime:anytime_calculate_result_exit
                    lttng enable-event --userspace anytime:rrt_star_iteration
                    lttng enable-event --userspace anytime:rrt_star_result
                    lttng enable-event --userspace anytime:rrt_star_init
                    lttng enable-event --userspace anytime:rrt_star_reset

                    lttng add-context --userspace --type=vpid
                    lttng add-context --userspace --type=vtid
                    lttng add-context --userspace --type=procname
                    lttng start

                    server_config="${CONFIG_DIR}/${config_name}_server.yaml"
                    client_config="${CONFIG_DIR}/${config_name}_client.yaml"

                    if [ "${thread_mode}" = "multi" ]; then
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
                    sleep ${RUN_DURATION}

                    lttng stop
                    sleep 1
                    kill ${LAUNCH_PID} 2>/dev/null || true
                    sleep 1
                    kill -9 ${LAUNCH_PID} 2>/dev/null || true
                    pkill -9 -f 'component_container' 2>/dev/null || true
                    pkill -9 -f 'ros2' 2>/dev/null || true
                    sleep 1
                    lttng destroy rrt_star_exp

                    if [ -d "${trace_output}" ] && [ "$(ls -A ${trace_output})" ]; then
                        echo "  ✓ Trace saved"
                    else
                        echo "  ✗ Warning: No trace data"
                    fi
                    sleep 1
                done
            done
        done
    done
done

echo ""
echo "Quick run completed! Running evaluation..."
cd "${EXPERIMENT_DIR}"
python3 evaluate_rrt_star.py
echo "Results saved to: ${RESULTS_DIR}"
