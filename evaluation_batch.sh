#!/bin/bash

# Array of batch sizes
batch_sizes=(100 200 500 1000 2000 5000)

# Loop through each batch size and call evaluation.sh
for batch_size in "${batch_sizes[@]}"; 
do
    ./evaluation.sh $batch_size
done
