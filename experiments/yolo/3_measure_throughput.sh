#!/bin/bash
#
# Step 3: Measure Maximum Throughput
#
# Purpose: Test different configurations to find maximum throughput
# Configurations: sync/async × single/multi-threaded (4 total)
# Each uses batch_size=25 (all layers, no cancellation)
# Output: traces/phase3_{sync|async}_{single|multi}_trial{1,2,3}/
#

set -e  # Exit on error

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
EXPERIMENT_DIR="${SCRIPT_DIR}"
TRACE_BASE_DIR="${EXPERIMENT_DIR}/traces"
NUM_TRIALS=1

# Test configurations: sync_mode, threading_mode, description
declare -a CONFIGS=(
    "sync:single:Sync+SingleThread"
    "sync:multi:Sync+MultiThread"
    "async:single:Async+SingleThread"
    "async:multi:Async+MultiThread"
)

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Step 3: Measure Maximum Throughput${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Testing configurations:"
echo "  - Batch size: 25 (all layers, no cancellation)"
echo "  - Mode: Proactive"
echo "  - Configurations: ${#CONFIGS[@]}"
echo "  - Trials per config: ${NUM_TRIALS}"
echo ""
for config in "${CONFIGS[@]}"; do
    IFS=':' read -r sync_mode threading_mode desc <<< "$config"
    echo "    - ${desc} (sync=${sync_mode}, multi_threading=${threading_mode})"
done
echo ""

# Source ROS2 environment
echo -e "${BLUE}Sourcing ROS2 environment...${NC}"
source "${WORKSPACE_DIR}/packages/install/setup.bash"

# Results summary
RESULTS_FILE="${EXPERIMENT_DIR}/results/phase3_timing_summary.txt"
mkdir -p "${EXPERIMENT_DIR}/results"
echo "Phase 3: Maximum Throughput Timing Results" > "${RESULTS_FILE}"
echo "========================================" >> "${RESULTS_FILE}"
echo "Date: $(date)" >> "${RESULTS_FILE}"
echo "" >> "${RESULTS_FILE}"

# Test each configuration
config_num=0
for config in "${CONFIGS[@]}"; do
    config_num=$((config_num + 1))
    IFS=':' read -r sync_mode threading_mode desc <<< "$config"
    
    echo ""
    echo -e "${CYAN}=======================================${NC}"
    echo -e "${CYAN}Configuration ${config_num}/${#CONFIGS[@]}: ${desc}${NC}"
    echo -e "${CYAN}=======================================${NC}"
    echo -e "  Sync mode: ${sync_mode}"
    echo -e "  Threading: ${threading_mode}"
    echo ""
    
    # Select appropriate server config file
    SERVER_CONFIG="${EXPERIMENT_DIR}/configs/phase3_server_${sync_mode}_${threading_mode}.yaml"
    CLIENT_CONFIG="${EXPERIMENT_DIR}/configs/phase3_client.yaml"
    
    if [ ! -f "${SERVER_CONFIG}" ]; then
        echo -e "${RED}Error: Server config not found: ${SERVER_CONFIG}${NC}"
        continue
    fi
    
    if [ ! -f "${CLIENT_CONFIG}" ]; then
        echo -e "${RED}Error: Client config not found: ${CLIENT_CONFIG}${NC}"
        continue
    fi
    
    # Log configuration to results file
    echo "Configuration: ${desc}" >> "${RESULTS_FILE}"
    echo "  - Sync mode: ${sync_mode}" >> "${RESULTS_FILE}"
    echo "  - Threading: ${threading_mode}" >> "${RESULTS_FILE}"
    echo "  - Server config: ${SERVER_CONFIG}" >> "${RESULTS_FILE}"
    echo "  - Client config: ${CLIENT_CONFIG}" >> "${RESULTS_FILE}"
    echo "" >> "${RESULTS_FILE}"
    
    # Run trials for this configuration
    declare -a trial_times=()
    
    for trial in $(seq 1 ${NUM_TRIALS}); do
        echo ""
        echo -e "${YELLOW}========================================${NC}"
        echo -e "${YELLOW}Config ${config_num}/${#CONFIGS[@]} - Trial ${trial}/${NUM_TRIALS}${NC}"
        echo -e "${YELLOW}========================================${NC}"
        
        TRACE_DIR="${TRACE_BASE_DIR}/phase3_${sync_mode}_${threading_mode}"
        TRIAL_TRACE_DIR="${TRACE_DIR}_trial${trial}"

        # Clean up old trace session and old trace data if exists
        echo -e "${BLUE}Cleaning up old LTTng session...${NC}"
        lttng destroy yolo_phase3 2>/dev/null || true
        rm -rf "${TRIAL_TRACE_DIR}"

        # Create LTTng session
        echo -e "${BLUE}Creating LTTng session...${NC}"
        lttng create yolo_phase3 --output="${TRIAL_TRACE_DIR}"
        
        # Enable all anytime events
        echo -e "${BLUE}Enabling tracepoints...${NC}"
        lttng enable-event -u 'anytime:*'
        
        # Start tracing
        echo -e "${BLUE}Starting trace session...${NC}"
        lttng start
        
        # Record start time
        START_TIME=$(date +%s)
        
        # Launch YOLO server and client components
        echo -e "${BLUE}Launching YOLO server and client...${NC}"
        echo "  - Batch size: 25 (all layers)"
        echo "  - Mode: Proactive"
        echo "  - Sync: ${sync_mode}"
        echo "  - Multi-threading: ${threading_mode}"
        
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
        
        # Record end time
        END_TIME=$(date +%s)
        ELAPSED_TIME=$((END_TIME - START_TIME))
        trial_times+=($ELAPSED_TIME)
        
        echo -e "${BLUE}Processing completed in ${ELAPSED_TIME} seconds, cleaning up...${NC}"
        
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
        pkill -9 -f 'anytime_yolo' 2>/dev/null || true
        pkill -9 -f 'video_publisher' 2>/dev/null || true
        pkill -9 -f 'ros2' 2>/dev/null || true
        
        echo -e "${GREEN}Trial ${trial} complete! Time: ${ELAPSED_TIME}s${NC}"
        echo -e "Trace saved to: ${TRIAL_TRACE_DIR}"
        
        # Log to results file
        echo "  Trial ${trial}: ${ELAPSED_TIME}s" >> "${RESULTS_FILE}"
        
        # Wait between trials
        if [ ${trial} -lt ${NUM_TRIALS} ]; then
            echo ""
            echo -e "${YELLOW}Waiting 5 seconds before next trial...${NC}"
            sleep 5
        fi
    done
    
    # Calculate statistics for this configuration
    if [ ${#trial_times[@]} -gt 0 ]; then
        total=0
        min_time=${trial_times[0]}
        max_time=${trial_times[0]}
        
        for time in "${trial_times[@]}"; do
            total=$((total + time))
            if [ $time -lt $min_time ]; then
                min_time=$time
            fi
            if [ $time -gt $max_time ]; then
                max_time=$time
            fi
        done
        
        avg_time=$((total / ${#trial_times[@]}))
        
        echo "" >> "${RESULTS_FILE}"
        echo "  Summary:" >> "${RESULTS_FILE}"
        echo "    Average: ${avg_time}s" >> "${RESULTS_FILE}"
        echo "    Min: ${min_time}s" >> "${RESULTS_FILE}"
        echo "    Max: ${max_time}s" >> "${RESULTS_FILE}"
        echo "" >> "${RESULTS_FILE}"
        
        echo ""
        echo -e "${GREEN}Configuration ${desc} Summary:${NC}"
        echo -e "  Average time: ${avg_time}s"
        echo -e "  Min time: ${min_time}s"
        echo -e "  Max time: ${max_time}s"
    fi
    
    echo "" >> "${RESULTS_FILE}"
    echo "----------------------------------------" >> "${RESULTS_FILE}"
    echo "" >> "${RESULTS_FILE}"
    
    # Wait between configurations
    if [ ${config_num} -lt ${#CONFIGS[@]} ]; then
        echo ""
        echo -e "${CYAN}Waiting 10 seconds before next configuration...${NC}"
        sleep 10
    fi
done

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Step 3 Complete: Throughput Measured${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "All configurations tested:"
for config in "${CONFIGS[@]}"; do
    IFS=':' read -r sync_mode threading_mode desc <<< "$config"
    echo "  - ${desc}: ${TRACE_BASE_DIR}/phase3_${sync_mode}_${threading_mode}_trial*"
done
echo ""
echo "Timing summary saved to: ${RESULTS_FILE}"
echo ""
echo "Results summary:"
cat "${RESULTS_FILE}"
echo ""
echo "Next step:"
echo "  4. Analyze throughput: python3 ${EXPERIMENT_DIR}/4_analyze_throughput.py"
echo ""
