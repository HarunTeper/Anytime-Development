#!/bin/bash
#
# Quick verification script to test new tracepoint implementation
# This runs a single trial and checks for the presence of all tracepoints
#

set -e

WORKSPACE_DIR="/home/vscode/workspace"
EXPERIMENT_DIR="${WORKSPACE_DIR}/experiments/yolo"
TRACE_DIR="${EXPERIMENT_DIR}/traces/verify_test"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}YOLO Tracepoint Verification Test${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Clean up old trace
echo -e "${BLUE}Cleaning up old traces...${NC}"
lttng destroy verify_test 2>/dev/null || true
rm -rf "${TRACE_DIR}"

# Source ROS2
echo -e "${BLUE}Sourcing ROS2 environment...${NC}"
source "${WORKSPACE_DIR}/packages/install/setup.bash"

# Create LTTng session
echo -e "${BLUE}Creating LTTng session...${NC}"
lttng create verify_test --output="${TRACE_DIR}"

# Enable all anytime events
echo -e "${BLUE}Enabling tracepoints...${NC}"
lttng enable-event -u 'anytime:*'

# Start tracing
echo -e "${BLUE}Starting trace...${NC}"
lttng start

# Launch YOLO server and client components
echo -e "${BLUE}Launching YOLO server and client...${NC}"
echo "  - Using default configuration"
echo "  - Batch size: 1"
echo "  - Mode: Proactive"

ros2 launch experiments yolo.launch.py &
YOLO_PID=$!

# Wait for components to initialize
echo -e "${BLUE}Waiting for components to initialize...${NC}"
sleep 3

# Launch video publisher
echo -e "${BLUE}Launching video publisher...${NC}"
ros2 launch experiments video_publisher_only.launch.py \
    image_path:=/home/vscode/workspace/packages/src/video_publisher/images &
VIDEO_PUB_PID=$!

# Wait for processing to complete (video publisher will exit when done)
echo -e "${BLUE}Processing images...${NC}"
sleep 10

# Stop trace
echo -e "${BLUE}Stopping trace...${NC}"
lttng stop
lttng destroy

# Clean up
kill ${VIDEO_PUB_PID} 2>/dev/null || true
kill ${YOLO_PID} 2>/dev/null || true
sleep 2
kill -9 ${VIDEO_PUB_PID} 2>/dev/null || true
kill -9 ${YOLO_PID} 2>/dev/null || true

# Kill any remaining YOLO processes
pkill -9 -f 'anytime_yolo' 2>/dev/null || true
pkill -9 -f 'video_publisher' 2>/dev/null || true
pkill -9 -f 'ros2' 2>/dev/null || true

# Verify tracepoints
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Tracepoint Verification Results${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

echo -e "${YELLOW}Checking for required tracepoints:${NC}"
echo ""

# Function to check tracepoint
check_tracepoint() {
    local name=$1
    local count=$(babeltrace "${TRACE_DIR}" 2>/dev/null | grep "anytime:${name}" | wc -l)
    if [ "$count" -gt 0 ]; then
        echo -e "✅ ${name}: ${GREEN}${count} events${NC}"
        return 0
    else
        echo -e "❌ ${name}: ${RED}NOT FOUND${NC}"
        return 1
    fi
}

# Check all YOLO tracepoints
all_good=true

check_tracepoint "yolo_init" || all_good=false
check_tracepoint "yolo_layer_start" || all_good=false
check_tracepoint "yolo_layer_end" || all_good=false
check_tracepoint "yolo_exit_calculation_start" || all_good=false
check_tracepoint "yolo_exit_calculation_end" || all_good=false
check_tracepoint "yolo_detection" || all_good=false
check_tracepoint "yolo_result" || all_good=false
check_tracepoint "yolo_reset" || all_good=false
check_tracepoint "yolo_image_processed" || all_good=false
check_tracepoint "anytime_base_activate" || all_good=false

echo ""
if [ "$all_good" = true ]; then
    echo -e "${GREEN}✅ All tracepoints verified successfully!${NC}"
    echo ""
    echo -e "${BLUE}Sample exit calculation events:${NC}"
    babeltrace "${TRACE_DIR}" 2>/dev/null | grep "yolo_exit_calculation_end" | head -3
    echo ""
    echo -e "${GREEN}Ready to run full Phase 1 experiments!${NC}"
else
    echo -e "${RED}⚠️  Some tracepoints are missing. Please review implementation.${NC}"
fi

echo ""
echo -e "${BLUE}Trace saved to: ${TRACE_DIR}${NC}"
