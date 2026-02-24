#!/bin/bash
#
# Step 0: Test Setup (Optional)
#
# Purpose: Quick test to verify your environment is configured correctly
# Tests: ROS2 workspace, YOLO weights, images, LTTng tracing
#

set -e  # Exit on error

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
EXPERIMENT_DIR="${SCRIPT_DIR}"
TRACE_DIR="${EXPERIMENT_DIR}/traces/test_single"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Step 0: Test Setup${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Source ROS2 environment
echo -e "${BLUE}Sourcing ROS2 environment...${NC}"
source "${WORKSPACE_DIR}/packages/install/setup.bash"

# Clean up old trace session and old trace data if exists
echo -e "${BLUE}Cleaning up old LTTng session...${NC}"
lttng destroy yolo_test 2>/dev/null || true
rm -rf "${TRACE_DIR}"

# Create LTTng session
echo -e "${BLUE}Creating LTTng session...${NC}"
lttng create yolo_test --output="${TRACE_DIR}"

# Enable all anytime events
echo -e "${BLUE}Enabling tracepoints...${NC}"
lttng enable-event -u 'anytime:*'

# Start tracing
echo -e "${BLUE}Starting trace session...${NC}"
lttng start

# Launch YOLO server and client components
echo -e "${BLUE}Launching YOLO server and client...${NC}"
echo "  - Batch size: 1 (default)"
echo "  - Mode: Proactive (default)"

ros2 launch experiments yolo.launch.py &
YOLO_PID=$!

# Wait for components to initialize
echo -e "${BLUE}Waiting for components to initialize (3 seconds)...${NC}"
sleep 3

# Launch video publisher
echo -e "${BLUE}Launching video publisher...${NC}"
echo ""
echo -e "${YELLOW}Processing will run for 30 seconds or until video publisher completes...${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop early if needed${NC}"

ros2 launch experiments video_publisher_only.launch.py \
    image_path:=${WORKSPACE_DIR}/packages/src/video_publisher/images &
VIDEO_PUB_PID=$!

# Wait for processing to complete or timeout after 30 seconds
echo -e "${BLUE}Waiting for processing to complete (max 30 seconds)...${NC}"
for i in {1..30}; do
    if ! kill -0 ${VIDEO_PUB_PID} 2>/dev/null; then
        echo -e "${GREEN}Processing completed!${NC}"
        break
    fi
    sleep 1
done

# Clean up
echo ""
echo -e "${BLUE}Test complete, cleaning up...${NC}"

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

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Test Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Trace saved to: ${TRACE_DIR}"
echo ""
echo "To verify trace data:"
echo "  babeltrace2 ${TRACE_DIR} | grep 'anytime:yolo' | head -20"
echo ""
echo "If you see YOLO tracepoints, the setup is working correctly!"
echo ""
