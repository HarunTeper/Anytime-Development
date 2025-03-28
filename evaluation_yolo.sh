#!/bin/bash

# Default operation mode: both running and plotting
mode="both"
num_runs=3  # Default number of runs

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
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  --run-only    Only run experiments without plotting"
            echo "  --plot-only   Only plot results without running experiments"
            echo "  --runs N      Number of runs for each configuration (default: 3)"
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

declare -a is_single_multi=("False")
declare -a is_reactive_proactive=("False")
declare -a is_passive_cooperative=("True")
declare -a is_sync_async=("False")
declare -a batch_sizes=(1)

# Create the results and plots directories
mkdir -p results/yolo
mkdir -p plots/yolo

# Run experiments if mode is "run" or "both"
if [[ "$mode" == "run" || "$mode" == "both" ]]; then
    echo "Running experiments with $num_runs runs per configuration..."
    for batch_size in "${batch_sizes[@]}"; do
        for threading in "${is_single_multi[@]}"; do
            for isReactiveProactive in "${is_reactive_proactive[@]}"; do
                for isPassiveCooperative in "${is_passive_cooperative[@]}"; do
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

                            # Map passive cooperative parameter
                            if [ "$isPassiveCooperative" = "True" ]; then
                                passive_param="cooperative"
                            else
                                passive_param="passive"
                            fi

                            # Map sync async parameter
                            if [ "$isSyncAsync" = "True" ]; then
                                sync_param="async"
                            else
                                sync_param="sync"
                            fi
                            
                            config_name="yolo_${batch_size}_${reactive_param}_${threading_param}_${passive_param}_${sync_param}_run${run}"
                            result_filename="yolo/yolo_raw_timestamps_batch_${batch_size}_${reactive_param}_${threading_param}_${passive_param}_${sync_param}_run${run}"
                            
                            echo "Running configuration: $config_name (Run $run of $num_runs)"

                            # Start the action server in the background
                            ros2 launch anytime_yolo action_server.launch.py multi_threading:=$threading is_reactive_proactive:=$reactive_param batch_size:=$batch_size is_passive_cooperative:=$passive_param is_sync_async:=$sync_param > ./results/yolo/${config_name}_server.log & server_pid=$!
                            
                            sleep 5
                            
                            # Start the action client in the background and pass result filename
                            ros2 launch anytime_yolo action_client.launch.py threading_type:=single result_filename:="${result_filename}" > "./results/yolo/${config_name}_client.log" & client_pid=$!
                            
                            # Wait for 60 seconds
                            sleep 60

                            # Terminate both processes after 60 seconds
                            kill $server_pid 2>/dev/null
                            kill $client_pid 2>/dev/null
                            # Force kill any remaining processes
                            pkill -9 -f 'anytime_yolo' 2>/dev/null
                            pkill -9 -f '/opt/ros/humble' 2>/dev/null
                            sleep 5
                        done
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
    
    # Prepare passive/cooperative arguments
    passive_args=""
    for isPassiveCooperative in "${is_passive_cooperative[@]}"; do
        if [ "$isPassiveCooperative" = "True" ]; then
            passive_args="$passive_args cooperative"
        else
            passive_args="$passive_args passive"
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
    python3 evaluation_plotter_yolo.py --threading $threading_args --reactive $reactive_args --passive-cooperative $passive_args --sync-async $sync_args --batch-sizes $batch_args --runs $num_runs --results-dir results/yolo --output-dir plots/yolo
fi
