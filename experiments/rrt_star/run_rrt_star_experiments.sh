#!/bin/bash
# RRT* Experimental Evaluation Script
# This script runs all RRT* configurations with LTTng tracing

set -e  # Exit on error

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

# Restart lttng-sessiond to ensure clean tracing state
pkill lttng-sessiond 2>/dev/null || true
sleep 2
lttng-sessiond --daemonize 2>/dev/null || true
sleep 1

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
EXPERIMENT_DIR="${SCRIPT_DIR}"
CONFIG_DIR="${EXPERIMENT_DIR}/configs"
TRACE_DIR="${EXPERIMENT_DIR}/traces"
RESULTS_DIR="${EXPERIMENT_DIR}/results"
PACKAGES_DIR="${WORKSPACE_DIR}/packages"

# Experiment parameters
BATCH_SIZES=(1 16 64 256 1024 4096 16384)
MODES=("reactive" "proactive")
THREADING=("single" "multi")
MAPS=("depot" "warehouse")
NUM_RUNS=1  # Number of trials per configuration

# Duration for each experiment run (in seconds)
RUN_DURATION=10

echo "========================================="
echo "RRT* Experimental Evaluation"
echo "========================================="
echo ""
echo "Configuration:"
echo "  - Batch sizes: ${BATCH_SIZES[*]}"
echo "  - Modes: ${MODES[*]}"
echo "  - Threading: ${THREADING[*]}"
echo "  - Maps: ${MAPS[*]}"
echo "  - Runs per config: ${NUM_RUNS}"
echo "  - Run duration: ${RUN_DURATION}s"
echo ""

# Source ROS2 workspace
cd "${PACKAGES_DIR}"
source install/setup.bash

# Resolve map directory from installed package
MAPS_DIR="$(ros2 pkg prefix anytime_rrt_star)/share/anytime_rrt_star/maps"
echo "Maps directory: ${MAPS_DIR}"

# Update MAPS_DIR placeholder in generated configs
if [ -d "${CONFIG_DIR}" ]; then
    echo "Updating map paths in config files..."
    find "${CONFIG_DIR}" -name "*_server.yaml" -exec sed -i "s|MAPS_DIR|${MAPS_DIR}|g" {} \;
fi

# Create directories
mkdir -p "${TRACE_DIR}"
mkdir -p "${RESULTS_DIR}"

# Counter for progress
total_configs=$((${#BATCH_SIZES[@]} * ${#MODES[@]} * ${#THREADING[@]} * ${#MAPS[@]} * NUM_RUNS))
current_config=0

# Iterate through all configurations
for batch_size in "${BATCH_SIZES[@]}"; do
    for mode in "${MODES[@]}"; do
        for thread_mode in "${THREADING[@]}"; do
            for map_name in "${MAPS[@]}"; do
                for run in $(seq 1 ${NUM_RUNS}); do
                    current_config=$((current_config + 1))

                    # Create config name
                    config_name="batch_${batch_size}_${mode}_${thread_mode}_${map_name}"
                    run_name="${config_name}_run${run}"

                    echo ""
                    echo "========================================="
                    echo "Running ${current_config}/${total_configs}: ${run_name}"
                    echo "========================================="

                    # Create trace directory for this run (remove old traces first)
                    trace_output="${TRACE_DIR}/${run_name}"
                    rm -rf "${trace_output}"
                    mkdir -p "${trace_output}"

                    # Cleanup any existing LTTng session
                    lttng destroy rrt_star_exp 2>/dev/null || true

                    # Create LTTng session
                    echo "  [1/5] Creating LTTng session..."
                    lttng create rrt_star_exp --output="${trace_output}"

                    # Enable tracepoints
                    echo "  [2/5] Enabling tracepoints..."

                    # Framework core tracepoints
                    lttng enable-event --userspace anytime:anytime_compute_entry
                    lttng enable-event --userspace anytime:anytime_compute_exit
                    lttng enable-event --userspace anytime:anytime_server_handle_cancel
                    lttng enable-event --userspace anytime:anytime_base_deactivate
                    lttng enable-event --userspace anytime:anytime_client_goal_sent
                    lttng enable-event --userspace anytime:anytime_client_cancel_sent
                    lttng enable-event --userspace anytime:anytime_client_goal_finished

                    # Overhead measurement tracepoints (required for overhead metrics)
                    lttng enable-event --userspace anytime:anytime_send_feedback_entry
                    lttng enable-event --userspace anytime:anytime_send_feedback_exit
                    lttng enable-event --userspace anytime:anytime_calculate_result_entry
                    lttng enable-event --userspace anytime:anytime_calculate_result_exit

                    # RRT*-specific tracepoints (subsampled via convergence_log_interval)
                    lttng enable-event --userspace anytime:rrt_star_iteration
                    lttng enable-event --userspace anytime:rrt_star_result
                    lttng enable-event --userspace anytime:rrt_star_init
                    lttng enable-event --userspace anytime:rrt_star_reset

                    # Add context information
                    lttng add-context --userspace --type=vpid
                    lttng add-context --userspace --type=vtid
                    lttng add-context --userspace --type=procname

                    # Start tracing
                    echo "  [3/5] Starting trace..."
                    lttng start

                    # Run the experiment
                    echo "  [4/5] Running experiment for ${RUN_DURATION}s..."

                    # Launch RRT* server and client
                    server_config="${CONFIG_DIR}/${config_name}_server.yaml"
                    client_config="${CONFIG_DIR}/${config_name}_client.yaml"

                    # Determine executor type
                    if [ "${thread_mode}" = "multi" ]; then
                        use_multi_threaded="true"
                    else
                        use_multi_threaded="false"
                    fi

                    # Launch the experiment in background
                    ros2 launch experiments rrt_star.launch.py \
                        server_config:="${server_config}" \
                        client_config:="${client_config}" \
                        use_multi_threaded:=${use_multi_threaded} \
                        log_level:=info &

                    LAUNCH_PID=$!

                    # Wait for experiment duration
                    sleep ${RUN_DURATION}

                    # Stop tracing (before killing processes to flush trace buffers)
                    echo "  [5/5] Stopping trace and saving..."
                    lttng stop
                    sleep 1

                    # Kill the launch process
                    kill ${LAUNCH_PID} 2>/dev/null || true
                    sleep 1
                    kill -9 ${LAUNCH_PID} 2>/dev/null || true

                    # Kill any remaining processes
                    pkill -9 -f 'component_container' 2>/dev/null || true
                    pkill -9 -f 'anytime_rrt_star' 2>/dev/null || true
                    pkill -9 -f 'ros2' 2>/dev/null || true
                    sleep 1

                    lttng destroy rrt_star_exp

                    # Verify trace was created
                    if [ -d "${trace_output}" ] && [ "$(ls -A ${trace_output})" ]; then
                        echo "  ✓ Trace saved to: ${trace_output}"
                    else
                        echo "  ✗ Warning: No trace data generated for ${run_name}"
                    fi

                    # Small delay between runs
                    sleep 1
                done
            done
        done
    done
done

echo ""
echo "========================================="
echo "All experiments completed!"
echo "========================================="
echo ""
echo "Traces saved to: ${TRACE_DIR}"
echo ""
echo "Now running evaluation script..."
echo ""

# Run the evaluation script
cd "${EXPERIMENT_DIR}"
python3 evaluate_rrt_star.py

echo ""
echo "========================================="
echo "Evaluation complete!"
echo "========================================="
echo "Results saved to: ${RESULTS_DIR}"
