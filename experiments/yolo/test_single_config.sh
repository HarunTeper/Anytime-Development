#!/bin/bash
#
# Test Single YOLO Configuration
# Purpose: Quick test to verify experiment setup works
#

set -e  # Exit on error

# Configuration
WORKSPACE_DIR="/home/vscode/workspace"
EXPERIMENT_DIR="${WORKSPACE_DIR}/experiments/yolo"
TRACE_DIR="${EXPERIMENT_DIR}/traces/test_single"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}YOLO Single Configuration Test${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Source ROS2 environment
echo -e "${BLUE}Sourcing ROS2 environment...${NC}"
source "${WORKSPACE_DIR}/packages/install/setup.bash"

# Clean up old trace session if exists
echo -e "${BLUE}Cleaning up old LTTng session...${NC}"
lttng destroy yolo_test 2>/dev/null || true

# Create LTTng session
echo -e "${BLUE}Creating LTTng session...${NC}"
lttng create yolo_test --output="${TRACE_DIR}"

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
echo -e "${BLUE}Launching YOLO server...${NC}"
echo "  - Batch size: 1"
echo "  - Mode: Proactive"
echo ""
echo -e "${YELLOW}Processing will run for 30 seconds or until video publisher completes...${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop early if needed${NC}"

ros2 launch anytime_yolo action_server.launch.py \
    is_reactive_proactive:=proactive \
    is_sync_async:=sync \
    batch_size:=1 \
    weights_path:=/home/vscode/workspace/packages/src/anytime_yolo/weights_32 &
SERVER_PID=$!

# Give server time to start
sleep 2

# Launch video publisher in background (starts publishing images)
echo -e "${BLUE}Launching video publisher...${NC}"
ros2 launch video_publisher video_publisher.launch.py \
    image_path:=/home/vscode/workspace/packages/src/video_publisher/images &
VIDEO_PUB_PID=$!

# Wait for video publisher to complete or timeout after 30 seconds
echo -e "${BLUE}Waiting for processing to complete (max 30 seconds)...${NC}"
for i in {1..30}; do
    if ! kill -0 ${VIDEO_PUB_PID} 2>/dev/null; then
        echo -e "${GREEN}Video publisher completed!${NC}"
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
