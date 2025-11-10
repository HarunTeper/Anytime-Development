# experiments

Unified launch files and configurations for anytime algorithm experiments.

## Launch Files

- `monte_carlo.launch.py` - Monte Carlo server + client
- `yolo.launch.py` - YOLO server + client
- `video_publisher_only.launch.py` - Standalone video publisher
- `interference.launch.py` - Monte Carlo + interference timer

All use component containers with configurable single/multi-threaded executors.

## Usage

```bash
# Basic usage
ros2 launch experiments monte_carlo.launch.py

# With custom configs
ros2 launch experiments yolo.launch.py \
    server_config:=/path/to/server.yaml \
    client_config:=/path/to/client.yaml \
    use_multi_threaded:=true \
    log_level:=info

# YOLO video publisher (separate)
ros2 launch experiments video_publisher_only.launch.py \
    image_path:=/path/to/images
```

## Common Launch Arguments

- `use_multi_threaded` - Executor type (default: `true`)
- `server_config` - Server YAML config path
- `client_config` - Client YAML config path
- `log_level` - ROS log level (debug/info/warn/error)

Default configurations are in `config/` directory.

See `/home/vscode/workspace/experiments/` for complete experimental evaluation pipelines.
