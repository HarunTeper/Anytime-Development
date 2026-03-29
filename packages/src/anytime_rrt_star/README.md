# anytime_rrt_star

RRT* (Rapidly-exploring Random Tree Star) as an anytime workload for the Anytime framework.

Based on **Karaman et al. (ICRA 2011) — "Anytime Motion Planning using the RRT\*"**. Implements standard RRT\* with branch-and-bound pruning on Nav2 occupancy grid maps.

## Architecture

The package mirrors the `anytime_monte_carlo` structure:

- **Action Server** — Runs RRT\* iterations in configurable blocks, reports best path cost as feedback
- **Action Client** — Sends iteration goals, receives feedback/results, controls cancel timing
- **Management Class** — Core RRT\* algorithm: sample, nearest, steer, collision-check, choose-parent, insert, rewire, prune

## Prerequisites

Build the workspace (from the workspace root):

```bash
source /opt/ros/humble/setup.bash  # or your ROS 2 distro
colcon build --packages-select anytime_rrt_star
source install/setup.bash
```

## Running

### Using launch files

**Server** (with default config):

```bash
ros2 launch anytime_rrt_star action_server.launch.py
```

**Server** with a map:

```bash
ros2 launch anytime_rrt_star action_server.launch.py \
  map_yaml_path:=/path/to/anytime_rrt_star/maps/depot.yaml \
  start_x:=5.0 start_y:=5.0 \
  goal_x:=25.0 goal_y:=10.0
```

**Client** (in a separate terminal):

```bash
ros2 launch anytime_rrt_star action_client.launch.py
```

### Using executables directly

```bash
# Server
ros2 run anytime_rrt_star anytime_rrt_server \
  --ros-args \
  -p map_yaml_path:=/path/to/maps/depot.yaml \
  -p start_x:=5.0 -p start_y:=5.0 \
  -p goal_x:=25.0 -p goal_y:=10.0 \
  -p block_size:=1024 \
  -p is_reactive_proactive:=reactive \
  --is_single_multi single \
  --log-level info

# Client (separate terminal)
ros2 run anytime_rrt_star anytime_rrt_client
```

### Using config file

Edit `config/server_params.yaml` with your desired parameters, then:

```bash
ros2 launch anytime_rrt_star action_server.launch.py \
  config_file:=/path/to/server_params.yaml
```

## Parameters

### Server Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `is_reactive_proactive` | string | `"reactive"` | Algorithm mode: `"reactive"` or `"proactive"` |
| `multi_threading` | bool | `false` | Enable multi-threading |
| `block_size` | int | `1024` | RRT\* iterations per block |
| `random_seed` | int | `42` | RNG seed for reproducibility |
| `map_yaml_path` | string | `""` | Path to Nav2 YAML map file |
| `start_x` | double | `5.0` | Start X position (meters) |
| `start_y` | double | `5.0` | Start Y position (meters) |
| `goal_x` | double | `25.0` | Goal X position (meters) |
| `goal_y` | double | `10.0` | Goal Y position (meters) |
| `step_size` | double | `0.5` | Max extension distance per steer (meters) |
| `goal_threshold` | double | `0.5` | Distance to consider goal reached (meters) |
| `goal_bias` | double | `0.05` | Probability of sampling goal directly |
| `gamma_rrt_star` | double | `0.0` | Near radius constant (0 = auto-compute from map) |
| `prune_interval` | int | `1000` | Branch-and-bound pruning interval (0 = disabled) |

### Client Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `goal_timer_period_ms` | int | `100` | Period between sending goals (ms) |
| `cancel_timeout_period_ms` | int | `50` | Time before cancelling active goal (ms) |

## Maps

Two Nav2 maps are included in `maps/`:

- **depot.yaml/pgm** — 604x307 pixels, 0.05 m/px, 30.2x15.4m. Small, mostly open layout.
- **warehouse.yaml/pgm** — 1006x1674 pixels, 0.03 m/px, 30.2x50.2m. Large structured environment with aisles.

Maps use the standard Nav2 PGM format with YAML metadata (resolution, origin, thresholds).

### Visualizing maps

A Python script is provided to render the maps with start/goal markers overlaid:

```bash
# Default: saves PNG images to experiments/rrt_star/results/plots/
python3 experiments/rrt_star/visualize_maps.py

# PDF output
python3 experiments/rrt_star/visualize_maps.py --format pdf

# Custom output directory
python3 experiments/rrt_star/visualize_maps.py --output-dir /tmp/map_plots

# Custom maps directory
python3 experiments/rrt_star/visualize_maps.py --maps-dir /path/to/maps
```

Requires `matplotlib` and `numpy` (`pip install matplotlib numpy`).

### Suggested start/goal for depot

```
start_x: 5.0, start_y: 5.0
goal_x: 25.0, goal_y: 10.0
```

### Suggested start/goal for warehouse

```
start_x: -10.0, start_y: -20.0
goal_x: 10.0, goal_y: 20.0
```

## Tests

```bash
colcon test --packages-select anytime_rrt_star
colcon test-result --verbose
```

Tests cover:
- OccupancyGrid PGM/YAML loading and collision checking
- RRT\* tree growth, steering, nearest-neighbor
- Path finding without obstacles
- Path finding on the depot map
- Branch-and-bound pruning
- Path cost improvement over time (anytime property)

## Action Interface

Defined in `anytime_interfaces/action/RrtStar.action`:

- **Goal:** `int32 goal` — number of RRT\* iterations to perform
- **Result:** `float32 result` (best path cost), `int32 iterations`, `int32 block_size`, `Duration block_time`
- **Feedback:** `float32 feedback` — current best path cost (infinity if no path found yet)
