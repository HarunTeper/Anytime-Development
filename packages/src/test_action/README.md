# Test Action Package

This is a standalone test package for testing ROS2 actions with multi-threaded executors.

## Package Contents

- **Action Definition**: `TestAction.action` - Simple action with goal_id, result, and progress feedback
- **Action Server**: Standalone action server using multi-threaded executor
- **Action Client**: Standalone action client that sends goals every 1 second

## Building

```bash
cd /home/vscode/workspace/packages
colcon build --packages-select test_action
source install/setup.bash
```

## Running

### Test 1: Start Client First, Then Server

Terminal 1 - Start the client:
```bash
source /home/vscode/workspace/packages/install/setup.bash
ros2 launch test_action action_client.launch.py
```

Terminal 2 - Start the server (after client is running):
```bash
source /home/vscode/workspace/packages/install/setup.bash
ros2 launch test_action action_server.launch.py
```

This tests whether the action client can handle the server not being available initially, and whether it works correctly once the server starts.

### Test 2: Normal Operation (Server First)

Terminal 1 - Start the server:
```bash
source /home/vscode/workspace/packages/install/setup.bash
ros2 launch test_action action_server.launch.py
```

Terminal 2 - Start the client:
```bash
source /home/vscode/workspace/packages/install/setup.bash
ros2 launch test_action action_client.launch.py
```

## Expected Behavior

- **Client**: Sends a new goal every 1 second, logs when goals are accepted/rejected, receives feedback and results
- **Server**: Accepts goals, executes placebo computation, publishes feedback every 500ms, returns result after ~2.5 seconds
- **Multi-threading**: Both client and server use multi-threaded executors to handle concurrent callbacks

## Features

- Completely independent of anytime_core
- Uses multi-threaded executor on both client and server
- Client continues sending goals even when server is not available
- **Server uses timer-based execution** (no separate threads spawned)
  - Timer is cancelled when idle
  - Timer is started when a goal is accepted
  - Timer is cancelled again when goal completes or is cancelled
  - All execution happens within the executor's thread pool
- Placebo compute function simulates work without actual computation
- Server rejects new goals while processing an existing goal

## Implementation Details

### Timer-Based Action Server

The action server uses a timer-based approach instead of spawning threads:

1. **Initialization**: A timer is created with 500ms interval but immediately cancelled
2. **Goal Acceptance**: When a goal is accepted, the timer is reset (started)
3. **Timer Callback**: Every 500ms the timer callback:
   - Performs placebo computation
   - Publishes feedback
   - Checks for cancellation requests
   - After 5 iterations (2.5 seconds total), succeeds the goal and cancels the timer
4. **Ready for Next Goal**: Timer remains cancelled until the next goal arrives

This design keeps all execution within the executor's thread pool and tests whether the multi-threaded executor can handle action callbacks properly without external threads.
