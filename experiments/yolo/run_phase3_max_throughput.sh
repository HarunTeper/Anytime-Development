#!/bin/bash
#
# YOLO Phase 3: Maximum Throughput Experiment Runner
# Purpose: Test maximum batch size to measure throughput limits
#

set -e  # Exit on error

# Configuration
WORKSPACE_DIR="/home/vscode/workspace"
EXPERIMENT_DIR="${WORKSPACE_DIR}/experiments/yolo"
CONFIG_FILE="${EXPERIMENT_DIR}/configs/phase3_max_throughput.yaml"
TRACE_DIR="${EXPERIMENT_DIR}/traces/phase3_max_throughput"
NUM_TRIALS=3

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}YOLO Phase 3: Maximum Throughput${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Configuration:"
echo "  - Batch size: 25 (all layers at once)"
echo "  - Mode: Proactive"
echo "  - Trials: ${NUM_TRIALS}"
echo "  - Trace output: ${TRACE_DIR}"
echo ""

# Check if config file exists
if [ ! -f "${CONFIG_FILE}" ]; then
    echo -e "${RED}Error: Configuration file not found: ${CONFIG_FILE}${NC}"
    exit 1
fi

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
    lttng destroy yolo_phase3_max_throughput 2>/dev/null || true
    
    # Create LTTng session
    echo -e "${BLUE}Creating LTTng session...${NC}"
    lttng create yolo_phase3_max_throughput --output="${TRIAL_TRACE_DIR}"
    
    # Enable all anytime events
    echo -e "${BLUE}Enabling tracepoints...${NC}"
    lttng enable-event -u 'anytime:*'
    
    # Start tracing
    echo -e "${BLUE}Starting trace session...${NC}"
    lttng start
    
    # Launch YOLO client in background
    echo -e "${BLUE}Launching YOLO client...${NC}"
    ros2 launch anytime_yolo action_client.launch.py \
        send_cancel:=false \
        cancel_delay_ms:=0 &
    CLIENT_PID=$!
    
    # Give client time to start
    sleep 2
    
    # Launch YOLO server in background
    echo -e "${BLUE}Launching YOLO server with maximum throughput configuration...${NC}"
    echo "  - Batch size: 25"
    echo "  - Mode: Proactive"
    echo "  - Sync: sync"
    
    ros2 launch anytime_yolo action_server.launch.py \
        is_reactive_proactive:=proactive \
        is_sync_async:=sync \
        batch_size:=25 \
        weights_path:=/home/vscode/workspace/packages/src/anytime_yolo/weights_32 &
    SERVER_PID=$!
    
    # Give server time to start
    sleep 2
    
    # Launch video publisher in background (starts publishing images)
    echo -e "${BLUE}Launching video publisher...${NC}"
    ros2 launch video_publisher video_publisher.launch.py \
        image_path:=/home/vscode/workspace/packages/src/video_publisher/images &
    VIDEO_PUB_PID=$!
    
    # Wait for video publisher to complete (it will shutdown when done)
    echo -e "${BLUE}Processing images... waiting for video publisher to complete${NC}"
    wait ${VIDEO_PUB_PID}
    
    echo -e "${BLUE}Video publisher completed, cleaning up...${NC}"
    
    # Stop trace
    echo -e "${BLUE}Stopping trace session...${NC}"
    lttng stop
    lttng destroy
    
    # Kill background processes
    echo -e "${BLUE}Stopping background processes...${NC}"
    kill ${SERVER_PID} 2>/dev/null || true
    kill ${CLIENT_PID} 2>/dev/null || true
    kill ${VIDEO_PUB_PID} 2>/dev/null || true
    
    # Wait for processes to stop
    sleep 2
    
    # Force kill if still running
    kill -9 ${SERVER_PID} 2>/dev/null || true
    kill -9 ${CLIENT_PID} 2>/dev/null || true
    kill -9 ${VIDEO_PUB_PID} 2>/dev/null || true
    
    # Kill any remaining YOLO processes
    pkill -9 -f 'anytime_yolo' 2>/dev/null || true
    pkill -9 -f 'video_publisher' 2>/dev/null || true
    pkill -9 -f 'ros2' 2>/dev/null || true
    
    echo -e "${GREEN}Trial ${trial} complete!${NC}"
    echo -e "Trace saved to: ${TRIAL_TRACE_DIR}"
    
    # Wait between trials
    if [ ${trial} -lt ${NUM_TRIALS} ]; then
        echo ""
        echo -e "${YELLOW}Waiting 5 seconds before next trial...${NC}"
        sleep 5
    fi
done

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Phase 3 Maximum Throughput Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Traces saved to: ${TRACE_DIR}_trial*"
echo ""
echo "Next steps:"
echo "  1. Run evaluation script: python3 ${EXPERIMENT_DIR}/evaluate_yolo.py"
echo ""
