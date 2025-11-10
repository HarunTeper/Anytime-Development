#!/bin/bash
#
# Step 6a: Test Single Cancellation Configuration
#
# Usage: ./6a_test_single_config.sh [config_name]
# Example: ./6a_test_single_config.sh bs1_proactive_sync_single
#
# Tests a single configuration before running the full Step 6 experiment suite.
#

set -e  # Exit on error

# Configuration
WORKSPACE_DIR="/home/vscode/workspace"
EXPERIMENT_DIR="${WORKSPACE_DIR}/experiments/yolo"
TRACE_BASE_DIR="${EXPERIMENT_DIR}/traces"

# Get config name from argument or use default
CONFIG_NAME=${1:-bs1_reactive_sync_single}

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Step 6a: Test Single Configuration${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Testing configuration: ${CONFIG_NAME}"
echo ""

# Config files
SERVER_CONFIG="${EXPERIMENT_DIR}/configs/phase4_server_${CONFIG_NAME}.yaml"
CLIENT_CONFIG="${EXPERIMENT_DIR}/configs/phase4_client.yaml"

# Check if config files exist
if [ ! -f "${SERVER_CONFIG}" ]; then
    echo -e "${RED}Error: Server config not found: ${SERVER_CONFIG}${NC}"
    echo ""
    echo "Available configurations:"
    ls -1 "${EXPERIMENT_DIR}/configs/phase4_server_"*.yaml | sed 's/.*phase4_server_/  - /' | sed 's/.yaml$//'
    exit 1
fi

if [ ! -f "${CLIENT_CONFIG}" ]; then
    echo -e "${RED}Error: Client config not found: ${CLIENT_CONFIG}${NC}"
    exit 1
fi

echo -e "${GREEN}✓${NC} Found server config: ${SERVER_CONFIG}"
echo -e "${GREEN}✓${NC} Found client config: ${CLIENT_CONFIG}"
echo ""

# Source ROS2 environment
echo -e "${BLUE}Sourcing ROS2 environment...${NC}"
source "${WORKSPACE_DIR}/packages/install/setup.bash"

# Trace directory
TRACE_DIR="${TRACE_BASE_DIR}/phase4_${CONFIG_NAME}_test"

echo ""
echo -e "${BLUE}Trace output: ${TRACE_DIR}${NC}"
echo ""

# Clean up old trace session if exists
echo -e "${BLUE}Cleaning up old LTTng session...${NC}"
lttng destroy yolo_phase4_test 2>/dev/null || true

# Create LTTng session
echo -e "${BLUE}Creating LTTng session...${NC}"
lttng create yolo_phase4_test --output="${TRACE_DIR}"

# Enable all anytime events
echo -e "${BLUE}Enabling tracepoints...${NC}"
lttng enable-event -u 'anytime:*'

# Start tracing
echo -e "${BLUE}Starting trace session...${NC}"
lttng start

echo ""
echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}Running Experiment${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""

# Launch YOLO server and client components
echo -e "${BLUE}Launching YOLO server and client...${NC}"
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
    image_path:=/home/vscode/workspace/packages/src/video_publisher/images &
VIDEO_PUB_PID=$!

# Wait for processing to complete (it will shutdown when done)
echo -e "${BLUE}Processing images... waiting for completion${NC}"
wait ${VIDEO_PUB_PID}

echo ""
echo -e "${GREEN}Processing completed!${NC}"
echo ""

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

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Test Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Trace saved to: ${TRACE_DIR}"
echo ""

# Quick verification
echo -e "${BLUE}Verifying trace...${NC}"
if [ -d "${TRACE_DIR}" ]; then
    EVENT_COUNT=$(babeltrace2 "${TRACE_DIR}" 2>/dev/null | grep -c "anytime:" || echo "0")
    echo -e "  Events captured: ${EVENT_COUNT}"
    
    if [ "${EVENT_COUNT}" -gt 0 ]; then
        echo -e "${GREEN}✓${NC} Trace looks good!"
        
        # Show some sample events
        echo ""
        echo "Sample events:"
        babeltrace2 "${TRACE_DIR}" 2>/dev/null | grep "anytime:" | head -10
    else
        echo -e "${RED}✗${NC} No events captured!"
    fi
else
    echo -e "${RED}✗${NC} Trace directory not found!"
fi

echo ""
echo "Next steps:"
echo "  1. Run full experiments: ./run_phase4_experiments.sh"
echo "  2. Analyze results: ./analyze_phase4.py"
echo ""
