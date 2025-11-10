#!/bin/bash
#
# Step 1: Collect Baseline Data
#
# Purpose: Run baseline configuration to collect layer-wise detection and timing data
# Configuration: batch_size=1, proactive, single-threaded, all 25 layers
# Output: traces/phase1_baseline_trial{1,2,3}/
#

set -e  # Exit on error

# Configuration
WORKSPACE_DIR="/home/vscode/workspace"
EXPERIMENT_DIR="${WORKSPACE_DIR}/experiments/yolo"
TRACE_DIR="${EXPERIMENT_DIR}/traces/phase1_baseline"
NUM_TRIALS=3

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Step 1: Collect Baseline Data${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Configuration:"
echo "  - Batch size: 1 (layer-by-layer)"
echo "  - Mode: Proactive"
echo "  - Threading: Single"
echo "  - Layers: All 25 layers"
echo "  - Trials: ${NUM_TRIALS}"
echo "  - Trace output: ${TRACE_DIR}"
echo ""

# Source ROS2 environment
echo -e "${BLUE}Sourcing ROS2 environment...${NC}"
source "${WORKSPACE_DIR}/packages/install/setup.bash"

# Run trials
for trial in $(seq 1 ${NUM_TRIALS}); do
    echo ""
    echo -e "${YELLOW}========================================${NC}"
    echo -e "${YELLOW}Trial ${trial}/${NUM_TRIALS}${NC}"
    echo -e "${YELLOW}========================================${NC}"
    
    TRIAL_TRACE_DIR="${TRACE_DIR}_trial${trial}"
    
    # Clean up old trace session if exists
    echo -e "${BLUE}Cleaning up old LTTng session...${NC}"
    lttng destroy yolo_phase1_baseline 2>/dev/null || true
    
    # Create LTTng session
    echo -e "${BLUE}Creating LTTng session...${NC}"
    lttng create yolo_phase1_baseline --output="${TRIAL_TRACE_DIR}"
    
    # Enable all anytime events
    echo -e "${BLUE}Enabling tracepoints...${NC}"
    lttng enable-event -u 'anytime:*'
    
    # Start tracing
    echo -e "${BLUE}Starting trace session...${NC}"
    lttng start
    
    # Launch YOLO server and client components with Phase 1 config
    echo -e "${BLUE}Launching YOLO server and client with baseline configuration...${NC}"
    echo "  - Batch size: 1"
    echo "  - Mode: Proactive"
    echo "  - Sync: sync"
    echo "  - Multi-threading: disabled"
    
    ros2 launch experiments yolo.launch.py \
        server_config:=${EXPERIMENT_DIR}/configs/phase1_server.yaml \
        client_config:=${EXPERIMENT_DIR}/configs/phase1_client.yaml &
    YOLO_PID=$!
    
    # Wait for components to initialize
    echo -e "${BLUE}Waiting for components to initialize (3 seconds)...${NC}"
    sleep 3
    
    # Launch video publisher
    echo -e "${BLUE}Launching video publisher...${NC}"
    ros2 launch experiments video_publisher_only.launch.py \
        image_path:=/home/vscode/workspace/packages/src/video_publisher/images &
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
    pkill -9 -f 'anytime_yolo' 2>/dev/null || true
    pkill -9 -f 'video_publisher' 2>/dev/null || true
    pkill -9 -f 'ros2' 2>/dev/null || true
    
    echo -e "${GREEN}Trial ${trial} complete!${NC}"
    echo -e "Trace saved to: ${TRIAL_TRACE_DIR}"
    
    # Wait between trials
    if [ ${trial} -lt ${NUM_TRIALS} ]; then
        echo ""
        echo -e "${YELLOW}Waiting 10 seconds before next trial...${NC}"
        sleep 5
    fi
done

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Step 1 Complete: Baseline Data Collected${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Traces saved to: ${TRACE_DIR}_trial*"
echo ""
echo "Next steps:"
echo "  2a. Analyze quality: python3 ${EXPERIMENT_DIR}/2a_analyze_quality.py"
echo "  2b. Analyze blocks: python3 ${EXPERIMENT_DIR}/2b_analyze_blocks.py"
echo ""
