#!/bin/bash
# Interference Experimental Evaluation Script
# This script runs all Interference configurations with LTTng tracing

set -e  # Exit on error

# Cleanup on interrupt
cleanup() {
    echo ""
    echo "Interrupted — cleaning up..."
    lttng stop 2>/dev/null || true
    lttng destroy interference_exp 2>/dev/null || true
    pkill -9 -f 'interference_timer' 2>/dev/null || true
    pkill -9 -f 'anytime_monte_carlo' 2>/dev/null || true
    pkill -9 -f 'ros2' 2>/dev/null || true
}
trap cleanup INT TERM

# Preflight checks
if ! command -v lttng &>/dev/null; then
    echo "ERROR: lttng not found. Install lttng-tools."
    exit 1
fi

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
EXPERIMENT_DIR="${SCRIPT_DIR}"
CONFIG_DIR="${EXPERIMENT_DIR}/configs"
TRACE_DIR="${EXPERIMENT_DIR}/traces"
RESULTS_DIR="${EXPERIMENT_DIR}/results"
PACKAGES_DIR="${WORKSPACE_DIR}/packages"

# Experiment parameters
BATCH_SIZES=(1024 2048 4096 8192 16384 32768 65536)
MODES=("reactive" "proactive")
THREADING=("single")
NUM_RUNS=1  # Number of trials per configuration

# Duration for each experiment run (in seconds)
RUN_DURATION=10

# Interference timer parameters (fixed)
TIMER_PERIOD_MS=100
EXECUTION_TIME_MS=10

echo "========================================="
echo "Interference Experimental Evaluation"
echo "========================================="
echo ""
echo "Configuration:"
echo "  - Batch sizes: ${BATCH_SIZES[*]}"
echo "  - Modes: ${MODES[*]}"
echo "  - Threading: ${THREADING[*]}"
echo "  - Runs per config: ${NUM_RUNS}"
echo "  - Run duration: ${RUN_DURATION}s"
echo ""
echo "Interference Timer Settings:"
echo "  - Timer period: ${TIMER_PERIOD_MS} ms"
echo "  - Execution time: ${EXECUTION_TIME_MS} ms"
echo ""

# Source ROS2 workspace
cd "${PACKAGES_DIR}"
source install/setup.bash

# Create directories
mkdir -p "${TRACE_DIR}"
mkdir -p "${RESULTS_DIR}"

# Counter for progress
total_configs=$((${#BATCH_SIZES[@]} * ${#MODES[@]} * ${#THREADING[@]} * NUM_RUNS))
current_config=0

# Iterate through all configurations
for batch_size in "${BATCH_SIZES[@]}"; do
    for mode in "${MODES[@]}"; do
        for thread_mode in "${THREADING[@]}"; do
            for run in $(seq 1 ${NUM_RUNS}); do
                current_config=$((current_config + 1))
                
                # Create config name
                config_name="batch_${batch_size}_${mode}_${thread_mode}"
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
                lttng destroy interference_exp 2>/dev/null || true
                
                # Create LTTng session
                echo "  [1/5] Creating LTTng session..."
                lttng create interference_exp --output="${trace_output}"
                
                # Enable selective tracepoints
                echo "  [2/5] Enabling tracepoints..."
                
                # Monte Carlo compute timing (only entry/exit for full compute batch)
                lttng enable-event --userspace anytime:anytime_compute_entry
                lttng enable-event --userspace anytime:anytime_compute_exit
                
                # Interference timer events (MAIN FOCUS)
                lttng enable-event --userspace anytime:interference_timer_init
                lttng enable-event --userspace anytime:interference_timer_callback_entry
                lttng enable-event --userspace anytime:interference_timer_callback_exit
                
                # Add context information
                lttng add-context --userspace --type=vpid
                lttng add-context --userspace --type=vtid
                lttng add-context --userspace --type=procname
                
                # Start tracing
                echo "  [3/5] Starting trace..."
                lttng start
                
                # Run the experiment
                echo "  [4/5] Running experiment for ${RUN_DURATION}s..."
                
                # Launch Interference experiment
                server_config="${CONFIG_DIR}/${config_name}_server.yaml"
                client_config="${CONFIG_DIR}/${config_name}_client.yaml"
                
                # Determine executor type
                if [ "${thread_mode}" = "multi" ]; then
                    use_multi_threaded="true"
                else
                    use_multi_threaded="false"
                fi
                
                # Launch the experiment in background
                ros2 launch experiments interference.launch.py \
                    server_config:="${server_config}" \
                    client_config:="${client_config}" \
                    use_multi_threaded:=${use_multi_threaded} \
                    timer_period_ms:=${TIMER_PERIOD_MS} \
                    execution_time_ms:=${EXECUTION_TIME_MS} \
                    log_level:=info  &
                
                LAUNCH_PID=$!
                
                # Wait for experiment duration
                sleep ${RUN_DURATION}
                
                # Kill the launch process
                kill ${LAUNCH_PID} 2>/dev/null || true
                sleep 1
                kill -9 ${LAUNCH_PID} 2>/dev/null || true
                
                # Kill any remaining interference processes
                pkill -9 -f 'interference_timer' 2>/dev/null || true
                pkill -9 -f 'ros2' 2>/dev/null || true
                
                # Give it a moment to flush
                sleep 2
                
                # Stop tracing
                echo "  [5/5] Stopping trace and saving..."
                lttng stop
                lttng destroy interference_exp
                
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
python3 evaluate_interference.py

echo ""
echo "========================================="
echo "Evaluation complete!"
echo "========================================="
echo "Results saved to: ${RESULTS_DIR}"
