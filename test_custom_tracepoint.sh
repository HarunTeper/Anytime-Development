#!/bin/bash
# Simple script to test custom tracepoints with LTTng

set -e

cd /home/vscode/workspace/packages
source install/setup.bash

echo "========================================="
echo "Custom Tracepoint Test"
echo "========================================="
echo ""

# Cleanup any existing session
lttng destroy test-custom 2>/dev/null || true

# Create session
echo "1. Creating LTTng session..."
lttng create test-custom --output=/tmp/test-custom-trace

# Enable custom tracepoint
echo "2. Enabling anytime:interference_timer_init..."
lttng enable-event --userspace 'anytime:interference_timer_init'

# Start tracing
echo "3. Starting trace..."
lttng start

# Run the node
echo "4. Running interference node (3 seconds)..."
timeout 3 ros2 run interference interference_timer_node \
  --ros-args -p timer_period_ms:=500 -p execution_time_ms:=5 &
NODE_PID=$!
wait $NODE_PID 2>/dev/null || true

# Stop tracing
echo "5. Stopping trace..."
lttng stop

# View results
echo ""
echo "========================================="
echo "Trace Results"
echo "========================================="
lttng view | grep "anytime:" || echo "No events captured"

# Count events
COUNT=$(lttng view | grep -c "anytime:" || echo "0")
echo ""
echo "Total custom events captured: $COUNT"

# Cleanup
lttng destroy test-custom

echo ""
echo "========================================="
echo "Test Complete!"
echo "========================================="
