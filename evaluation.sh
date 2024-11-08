#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <batch_size>"
    exit 1
fi

batch_size=$1

source packages/install/setup.bash

declare -a threading_types=("True")
declare -a anytime_actives=("False")
declare -a separate_threads=("True")

for threading in "${threading_types[@]}"; do
    for active in "${anytime_actives[@]}"; do
        for separate in "${separate_threads[@]}"; do
            config_name="anytime_${threading}_${active}_${separate}_${batch_size}"
            echo "Running configuration: $config_name"

            # Start the action server in the background
            ros2 launch anytime_monte_carlo action_server.launch.py multi_threading:=$threading anytime_active:=$active separate_thread:=$separate batch_size:=$batch_size > /dev/null &
            server_pid=$!
            
            sleep 5
            
            # Start the action client in the background and log output
            ros2 launch anytime_monte_carlo action_client.launch.py > "./results/${config_name}.log" &
            client_pid=$!
            
            # Wait for 300 seconds
            sleep 30

            # Terminate both processes after 300 seconds
            kill $server_pid 2>/dev/null
            kill $client_pid 2>/dev/null
            pkill -f '/opt/ros/humble'
            
            # Wait briefly to ensure processes have terminated
            sleep 5

        done
    done
done

python3 evaluation_plotter.py $batch_size