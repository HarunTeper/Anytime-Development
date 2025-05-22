#!/bin/bash

# Default operation mode: both running and plotting
mode="both"
num_runs=3  # Default number of runs
debug_flag="false"

# Add cancel_layer_score flag
cancel_layer_score_flag="true"
declare -a is_reactive_proactive=("True")

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        --run-only)
            mode="run"
            shift
            ;;
        --plot-only)
            mode="plot"
            shift
            ;;
        --runs)
            num_runs="$2"
            shift
            shift
            ;;
        --debug)
            debug_flag="true"
            shift
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  --run-only    Only run experiments without plotting"
            echo "  --plot-only   Only plot results without running experiments"
            echo "  --runs N      Number of runs for each configuration (default: 3)"
            echo "  --debug       Enable debug logging in ROS2 nodes"
            echo "  --help        Display this help message"
            echo "Default behavior: Run experiments and plot results"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

source packages/install/setup.bash

declare -a is_single_multi=("False" "True")
declare -a is_sync_async=("False" "True")
declare -a batch_sizes=(1 5 10)

# Create the results and plots directories
mkdir -p results/yolo
mkdir -p plots/yolo

# Run experiments if mode is "run" or "both"
if [[ "$mode" == "run" || "$mode" == "both" ]]; then
    echo "Running experiments with $num_runs runs per configuration..."
    for batch_size in "${batch_sizes[@]}"; do
        for threading in "${is_single_multi[@]}"; do
            for isReactiveProactive in "${is_reactive_proactive[@]}"; do
                for isSyncAsync in "${is_sync_async[@]}"; do
                    for run in $(seq 1 $num_runs); do
                        
                        # Map reactive parameter value
                        if [ "$isReactiveProactive" = "True" ]; then
                            reactive_param="proactive"
                        else
                            reactive_param="reactive"
                        fi
                        
                        # Map threading parameter value
                        if [ "$threading" = "True" ]; then
                            threading_param="multi"
                        else
                            threading_param="single"
                        fi

                        # Map sync async parameter
                        if [ "$isSyncAsync" = "True" ]; then
                            sync_param="async"
                        else
                            sync_param="sync"
                        fi
                        
                        config_name="yolo_${batch_size}_${reactive_param}_${threading_param}_${sync_param}_run${run}"
                        result_filename="yolo/yolo_raw_timestamps_batch_${batch_size}_${reactive_param}_${threading_param}_${sync_param}_run${run}"
                        
                        echo "Running configuration: $config_name (Run $run of $num_runs)"

                        # Start the action server in the background
                        ros2 launch anytime_yolo action_server.launch.py multi_threading:=$threading is_reactive_proactive:=$reactive_param batch_size:=$batch_size is_sync_async:=$sync_param debug:=$debug_flag > ./results/yolo/${config_name}_server.log & server_pid=$!

                        sleep 5

                        # Start the action client in the background and pass result filename and cancel_layer_score
                        ros2 launch anytime_yolo action_client.launch.py threading_type:=single result_filename:="${result_filename}" debug:=$debug_flag cancel_layer_score:=$cancel_layer_score_flag > "./results/yolo/${config_name}_client.log" & client_pid=$!

                        # Start the detection visualizer in the background
                        ros2 launch video_publisher detection_visualizer.launch.py > ./results/yolo/${config_name}_visualizer.log & visualizer_pid=$!
                        
                        sleep 5

                        # Run the video publisher (blocking, foreground)
                        ros2 launch video_publisher video_publisher.launch.py > ./results/yolo/${config_name}_publisher.log

                        # Wait for video_publisher process to finish if still running
                        while ps -p $(pgrep -f "video_publisher.launch.py") > /dev/null 2>&1; do
                            sleep 1
                        done

                        # Terminate the processes after the video publisher finishes
                        kill $server_pid 2>/dev/null
                        kill $client_pid 2>/dev/null
                        kill $visualizer_pid 2>/dev/null
                        kill $publisher_pid 2>/dev/null

                        # Wait for specific processes to terminate
                        for process in 'anytime_yolo' 'video_publisher' 'detection_visualizer'; do
                            while pgrep -f "$process" > /dev/null 2>&1; do
                                pkill -9 -f 'anytime_yolo' 2>/dev/null
                                pkill -9 -f 'video_publisher' 2>/dev/null
                                pkill -9 -f 'detection_visualizer' 2>/dev/null
                                pkill -9 -f '/opt/ros/humble' 2>/dev/null
                                sleep 1
                            done
                        done

                        sleep 1
                    done
                done
            done
        done
    done
fi

# Plot results if mode is "plot" or "both"
if [[ "$mode" == "plot" || "$mode" == "both" ]]; then
    echo "Plotting results combining data from $num_runs runs..."
    
    # Prepare threading arguments
    threading_args=""
    for threading in "${is_single_multi[@]}"; do
        # Map threading parameter value
        if [ "$threading" = "True" ]; then
            threading_args="$threading_args multi"
        else
            threading_args="$threading_args single"
        fi
    done
    
    # Prepare reactive arguments
    reactive_args=""
    for isReactiveProactive in "${is_reactive_proactive[@]}"; do
        # Map reactive parameter value
        if [ "$isReactiveProactive" = "True" ]; then
            reactive_args="$reactive_args proactive"
        else
            reactive_args="$reactive_args reactive"
        fi
    done
    
    # Prepare sync/async arguments
    sync_args=""
    for isSyncAsync in "${is_sync_async[@]}"; do
        if [ "$isSyncAsync" = "True" ]; then
            sync_args="$sync_args async"
        else
            sync_args="$sync_args sync"
        fi
    done
    
    # Prepare batch size arguments
    batch_args=""
    for batch_size in "${batch_sizes[@]}"; do
        batch_args="$batch_args $batch_size"
    done
    
    echo "Running plotter with all configurations..."
    python3 evaluation_plotter_yolo.py --threading $threading_args --reactive $reactive_args --sync-async $sync_args --batch-sizes $batch_args --runs $num_runs --results-dir results/yolo --output-dir plots/yolo
fi
