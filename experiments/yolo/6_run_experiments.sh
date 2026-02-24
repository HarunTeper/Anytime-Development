#!/bin/bash
#
# Step 6: Run Cancellation Experiments
#
# Purpose: Test cancellation performance across different configurations
# Configurations: 3 block sizes × 1 mode × 2 sync × 2 threading = 12 configs
# Client cancellation: After 16 layers OR score threshold ≥ 0.8
# Output: traces/phase4_bs{1,8,25}_proactive_{sync|async}_{single|multi}_trial{1,2,3}/
#

set -e  # Exit on error

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
EXPERIMENT_DIR="${SCRIPT_DIR}"
TRACE_BASE_DIR="${EXPERIMENT_DIR}/traces"
NUM_TRIALS=1

# Test parameters
BLOCK_SIZES=(1 8 16 25)
MODES=("proactive")
SYNC_MODES=("sync" "async")
THREADING_MODES=("single" "multi")

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Step 6: Run Cancellation Experiments${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Testing configurations:"
echo "  - Block sizes: ${BLOCK_SIZES[@]}"
echo "  - Modes: ${MODES[@]}"
echo "  - Sync modes: ${SYNC_MODES[@]}"
echo "  - Threading modes: ${THREADING_MODES[@]}"
echo "  - Trials per config: ${NUM_TRIALS}"
echo ""
echo "Client cancellation settings:"
echo "  - Cancel after: 16 layers"
echo "  - Score threshold: ≥ 0.8"
echo "  - Target class: 9 (traffic light)"
echo ""

# Calculate total configurations
TOTAL_CONFIGS=$((${#BLOCK_SIZES[@]} * ${#MODES[@]} * ${#SYNC_MODES[@]} * ${#THREADING_MODES[@]}))
echo "Total configurations: ${TOTAL_CONFIGS}"
echo "Total experiments: $((TOTAL_CONFIGS * NUM_TRIALS))"
echo ""

# Source ROS2 environment
echo -e "${BLUE}Sourcing ROS2 environment...${NC}"
source "${WORKSPACE_DIR}/packages/install/setup.bash"

# Results summary
RESULTS_FILE="${EXPERIMENT_DIR}/results/phase4_experiment_summary.txt"
mkdir -p "${EXPERIMENT_DIR}/results"
echo "Phase 4: Final Cancellation Experiment Results" > "${RESULTS_FILE}"
echo "========================================" >> "${RESULTS_FILE}"
echo "Date: $(date)" >> "${RESULTS_FILE}"
echo "" >> "${RESULTS_FILE}"
echo "Client Configuration:" >> "${RESULTS_FILE}"
echo "  - Cancel after layers: 16" >> "${RESULTS_FILE}"
echo "  - Score threshold: 0.8" >> "${RESULTS_FILE}"
echo "  - Target class: 9 (traffic light)" >> "${RESULTS_FILE}"
echo "" >> "${RESULTS_FILE}"

CLIENT_CONFIG="${EXPERIMENT_DIR}/configs/phase4_client.yaml"

if [ ! -f "${CLIENT_CONFIG}" ]; then
    echo -e "${RED}Error: Client config not found: ${CLIENT_CONFIG}${NC}"
    exit 1
fi

# Test each configuration
config_num=0

for block_size in "${BLOCK_SIZES[@]}"; do
    for mode in "${MODES[@]}"; do
        for sync_mode in "${SYNC_MODES[@]}"; do
            for threading_mode in "${THREADING_MODES[@]}"; do
                config_num=$((config_num + 1))
                
                CONFIG_NAME="bs${block_size}_${mode}_${sync_mode}_${threading_mode}"
                
                echo ""
                echo -e "${CYAN}=======================================${NC}"
                echo -e "${CYAN}Configuration ${config_num}/${TOTAL_CONFIGS}: ${CONFIG_NAME}${NC}"
                echo -e "${CYAN}=======================================${NC}"
                echo -e "  Block size: ${block_size}"
                echo -e "  Mode: ${mode}"
                echo -e "  Sync: ${sync_mode}"
                echo -e "  Threading: ${threading_mode}"
                echo ""
                
                # Server config file
                SERVER_CONFIG="${EXPERIMENT_DIR}/configs/phase4_server_${CONFIG_NAME}.yaml"
                
                if [ ! -f "${SERVER_CONFIG}" ]; then
                    echo -e "${RED}Error: Server config not found: ${SERVER_CONFIG}${NC}"
                    continue
                fi
                
                # Log configuration to results file
                echo "Configuration ${config_num}/${TOTAL_CONFIGS}: ${CONFIG_NAME}" >> "${RESULTS_FILE}"
                echo "  - Block size: ${block_size}" >> "${RESULTS_FILE}"
                echo "  - Mode: ${mode}" >> "${RESULTS_FILE}"
                echo "  - Sync: ${sync_mode}" >> "${RESULTS_FILE}"
                echo "  - Threading: ${threading_mode}" >> "${RESULTS_FILE}"
                echo "" >> "${RESULTS_FILE}"
                
                # Run trials for this configuration
                for trial in $(seq 1 ${NUM_TRIALS}); do
                    echo ""
                    echo -e "${YELLOW}========================================${NC}"
                    echo -e "${YELLOW}Config ${config_num}/${TOTAL_CONFIGS} - Trial ${trial}/${NUM_TRIALS}${NC}"
                    echo -e "${YELLOW}${CONFIG_NAME}${NC}"
                    echo -e "${YELLOW}========================================${NC}"
                    
                    TRACE_DIR="${TRACE_BASE_DIR}/phase4_${CONFIG_NAME}"
                    TRIAL_TRACE_DIR="${TRACE_DIR}_trial${trial}"

                    # Clean up old trace session and old trace data if exists
                    echo -e "${BLUE}Cleaning up old LTTng session...${NC}"
                    lttng destroy yolo_phase4 2>/dev/null || true
                    rm -rf "${TRIAL_TRACE_DIR}"

                    # Create LTTng session
                    echo -e "${BLUE}Creating LTTng session...${NC}"
                    lttng create yolo_phase4 --output="${TRIAL_TRACE_DIR}"
                    
                    # Enable all anytime events
                    echo -e "${BLUE}Enabling tracepoints...${NC}"
                    lttng enable-event -u 'anytime:*'
                    
                    # Start tracing
                    echo -e "${BLUE}Starting trace session...${NC}"
                    lttng start
                    
                    # Launch YOLO server and client components
                    echo -e "${BLUE}Launching YOLO server and client...${NC}"
                    echo "  - Server config: ${SERVER_CONFIG}"
                    echo "  - Client config: ${CLIENT_CONFIG}"
                    
                    ros2 launch experiments yolo.launch.py \
                        server_config:=${SERVER_CONFIG} \
                        client_config:=${CLIENT_CONFIG} &
                    YOLO_PID=$!
                    
                    # Wait for components to initialize
                    echo -e "${BLUE}Waiting for components to initialize (3 seconds)...${NC}"
                    sleep 3
                    
                    # Launch video publisher
                    echo -e "${BLUE}Launching video publisher...${NC}"
                    ros2 launch experiments video_publisher_only.launch.py \
                        image_path:=${WORKSPACE_DIR}/packages/src/video_publisher/images &
                    VIDEO_PUB_PID=$!
                    
                    # Wait for processing to complete (it will shutdown when done)
                    echo -e "${BLUE}Processing images... waiting for completion${NC}"
                    wait ${VIDEO_PUB_PID}
                    
                    echo -e "${BLUE}Processing completed, cleaning up...${NC}"
                    
                    # Stop trace
                    echo -e "${BLUE}Stopping trace session...${NC}"
                    lttng stop
                    lttng destroy
                    
                    # Kill background processes
                    echo -e "${BLUE}Stopping background processes...${NC}"
                    kill ${VIDEO_PUB_PID} 2>/dev/null || true
                    kill ${YOLO_PID} 2>/dev/null || true
                    
                    # Wait for processes to stop
                    sleep 2
                    
                    # Force kill if still running
                    kill -9 ${VIDEO_PUB_PID} 2>/dev/null || true
                    kill -9 ${YOLO_PID} 2>/dev/null || true
                    
                    # Kill any remaining YOLO processes
                    pkill -9 -f 'component_container' 2>/dev/null || true
                    pkill -9 -f 'anytime_yolo' 2>/dev/null || true
                    pkill -9 -f 'video_publisher' 2>/dev/null || true
                    pkill -9 -f 'ros2' 2>/dev/null || true
                    
                    echo -e "${GREEN}Trial ${trial} complete!${NC}"
                    echo -e "Trace saved to: ${TRIAL_TRACE_DIR}"
                    
                    # Log to results file
                    echo "  Trial ${trial}: Complete" >> "${RESULTS_FILE}"
                    
                    # Wait between trials
                    if [ ${trial} -lt ${NUM_TRIALS} ]; then
                        echo ""
                        echo -e "${YELLOW}Waiting 5 seconds before next trial...${NC}"
                        sleep 5
                    fi
                done
                
                echo "" >> "${RESULTS_FILE}"
                echo "----------------------------------------" >> "${RESULTS_FILE}"
                echo "" >> "${RESULTS_FILE}"
                
                # Wait between configurations
                if [ ${config_num} -lt ${TOTAL_CONFIGS} ]; then
                    echo ""
                    echo -e "${MAGENTA}Waiting 10 seconds before next configuration...${NC}"
                    sleep 10
                fi
            done
        done
    done
done

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Step 6 Complete: Cancellation Experiments Done${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Tested ${TOTAL_CONFIGS} configurations × ${NUM_TRIALS} trials = $((TOTAL_CONFIGS * NUM_TRIALS)) experiments"
echo ""
echo "Traces saved to: ${TRACE_BASE_DIR}/phase4_*"
echo "Summary saved to: ${RESULTS_FILE}"
echo ""
echo "Next step:"
echo "  7. Analyze cancellation: python3 ${EXPERIMENT_DIR}/7_analyze_cancellation.py"
echo ""
