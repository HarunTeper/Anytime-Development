import os

import matplotlib.pyplot as plt

import numpy as np
import pandas as pd
import sys

def plot_data(threading, reactive, separate, batch_size):
    # Define the configurations
    threading_types = ["False", "True"] if threading == "Both" else [threading]
    anytime_reactives = ["False", "True"] if reactive == "Both" else [reactive]
    separate_threads = ["False", "True"] if separate == "Both" else [separate]

    # Initialize a dictionary to store the data
    data = {}

    # Read the data from the files
    for threading in threading_types:
        for reactive in anytime_reactives:
            for separate in separate_threads:
                config_name = f"anytime_{threading}_{reactive}_{separate}_{batch_size}"
                file_path = f"/home/vscode/workspace/results/{config_name}.log"
                
                # map threading from false true to single multi
                # map reactive to false true proactive reactive
                # map false true to executor pthread
                # create the key_name using the mapped values
                
                if threading == "False":
                    threading_mapped = "single"
                else:
                    threading_mapped = "multi"
                    
                if reactive == "False":
                    reactive_mapped = "proactive"
                else:
                    reactive_mapped = "reactive"
                    
                if separate == "False":
                    separate_mapped = "executor"
                else:
                    separate_mapped = "pthread"
                    
                key_name = f"{threading_mapped}_{reactive_mapped}_{separate_mapped}"
                
                value_dict = {}
                
                print(f"Checking file {file_path}")
                if os.path.exists(file_path):
                    with open(file_path, 'r') as file:
                        
                        # determine the last line that has "Number of iterations" in it
                        
                        lines = file.readlines()
                        last_iteration_line = None
                        for line in lines:
                            if "Number of iterations: mean=" in line:
                                last_iteration_line = line
                        if last_iteration_line:
                            lines = lines[:lines.index(last_iteration_line)+1]
        
                        
                            
            
                        
                        lines = lines[-7:]
                        
                                                
                        print(lines)
                        
                        
                        
                        for line in lines:
                            print(line)
                            # skip the line if it is empty
                            if not line:
                                continue
                            print(line)
                            print(line)
                            print(line)
                            print(line)
                            
                            name = line.split(":")[1][1:]
                            print(file_path)
                            print(name)
                            print("AAAAA")
                            print(line)
                            
                            values = line.split(":")[2].split(" ")
                            print(values)
                            
                            # remove any values from values that do not include an equal sign
                            values = [value for value in values if "=" in value]
                            
                            # remove any commas from the values
                            values = [value.replace(",", "") for value in values]
                            
                            print(name)
                            print(values)
                            
                            for value in values:
                                print(value)
                            
                            mean = values[0].split("=")[1]
                            stdev = values[1].split("=")[1]
                            p99 = values[2].split("=")[1]
                            p999 = values[3].split("=")[1]
                            max_value = values[4].split("=")[1]
                            
                            print("Mean:", mean)
                            print("Stdev:", stdev)
                            print("P99:", p99)
                            print("P999:", p999)
                            print("Max:", max_value)
                            
                            # create a dictionary to store the values
                            value_dict[name] = {
                                "mean": float(mean),
                                "stdev": float(stdev),
                                "p99": float(p99),
                                "p999": float(p999),
                                "max": float(max_value)
                            }
                    data[key_name] = value_dict
    
    print(data)
    
    # the values for everything except iterations are in ms, convert to seconds
    for config in data:
        for interval in data[config]:
            if interval != 'Number of iterations':
                data[config][interval]["mean"] = data[config][interval]["mean"] / 1000
                data[config][interval]["stdev"] = data[config][interval]["stdev"] / 1000
                data[config][interval]["p99"] = data[config][interval]["p99"] / 1000
                data[config][interval]["p999"] = data[config][interval]["p999"] / 1000
                data[config][interval]["max"] = data[config][interval]["max"] / 1000
    
    # print number of keys
    print(len(data.keys()))

    # Extract intervals and configurations
    intervals = list(data[next(iter(data))].keys())  # Get intervals from any configuration
    configs = list(data.keys())  # All configurations
    
    print(intervals)
    print(configs)

    # Colors for different statistics
    colors = {
        "mean": "skyblue",
        "stdev": "orange",
        "p99": "lightgreen",
        "p999": "purple",
        "max": "salmon"
    }

    # Create subplots, one for each interval
    fig, axs = plt.subplots(len(intervals), 1, figsize=(12, 4 * len(intervals)))
    fig.tight_layout(pad=10.0)

    # Iterate over intervals to create a plot for each
    for i, interval in enumerate(intervals):
        ax = axs[i] if len(intervals) > 1 else axs  # Handle single plot case
        
        # Collect data for each configuration for the current interval
        means = [data[config][interval]["mean"] for config in configs]
        stdevs = [data[config][interval]["stdev"] for config in configs]
        p99s = [data[config][interval]["p99"] for config in configs]
        p999s = [data[config][interval]["p999"] for config in configs]
        maxs = [data[config][interval]["max"] for config in configs]

        # Define bar width and positions for grouped bars
        bar_width = 0.2
        indices = np.arange(len(configs))
        
        max_value = max(maxs)
        
        # set the y bar limit between 0 and the max value
        ax.set_ylim(0, max_value + 0.0001)
        
        # Plot bars for each statistic
        ax.bar(indices - 1.5 * bar_width, means, bar_width, label="Mean", color=colors["mean"])
        ax.bar(indices - 0.5 * bar_width, stdevs, bar_width, label="Std Dev", color=colors["stdev"])
        ax.bar(indices + 0.5 * bar_width, p99s, bar_width, label="P99", color=colors["p99"])
        ax.bar(indices + 1.5 * bar_width, p999s, bar_width, label="P999", color=colors["p999"])
        ax.bar(indices + 2.5 * bar_width, maxs, bar_width, label="Max", color=colors["max"])

        # Labeling the bars
        ax.set_title(interval)
        ax.set_xticks(indices)
        ax.set_xticklabels(configs, rotation=45, ha="right")
        ax.set_ylabel("Value")
        ax.legend()

    # save the plot
    plt.savefig("/home/vscode/workspace/plots/evaluation_plot_"+str(batch_size)+".png")
    
    
    # Extract configuration names and intervals from the data
    configurations = list(data.keys())
    intervals = [key for key in data[configurations[0]].keys()]
    
    # filter out the max values that are over the p999 values
    for config in configurations:
        for interval in intervals:
            if data[config][interval]["max"] > data[config][interval]["p999"]:
                data[config][interval]["max"] = data[config][interval]["p999"]

    # Organize data for boxplots
    interval_data = {interval: [] for interval in intervals}
    for interval in intervals:
        for config in configurations:
            stats = data[config][interval]
            # For boxplot, we will consider mean, p99, p999, and max
            interval_data[interval].append([stats['mean'], stats['p99'], stats['p999'], stats['max']])

    # Plotting
    fig, ax = plt.subplots(len(intervals), 1, figsize=(15, 3 * len(intervals)), sharex=True)
    fig.suptitle('Boxplots for Each Interval by Configuration', fontsize=16)

    for idx, interval in enumerate(intervals):
        # Boxplot data for the current interval across all configurations
        ax[idx].boxplot(interval_data[interval], tick_labels=configurations, patch_artist=True)
        ax[idx].set_title(interval)
        ax[idx].set_ylabel('Latency (s)')
        ax[idx].set_ylim(0, max([max(data) for data in interval_data[interval]]) + 0.0001)
        

    plt.xticks(rotation=45, ha="right")
    plt.xlabel('Configuration')
    plt.tight_layout(rect=[0, 0, 1, 0.95])

    # save the plot
    plt.savefig("/home/vscode/workspace/plots/evaluation_boxplot_"+str(batch_size)+".png")
                
def main():
    
    if len(sys.argv) < 4:
        print("Usage: python evaluation_plotter.py <executor_types> <anytime_type> <separate_threads> <batch_size>")
        sys.exit(1)

    executor_types = sys.argv[1]
    anytime_type = sys.argv[2]
    separate_threads = sys.argv[3]
    batch_size = int(sys.argv[4])
    
    print(f"Executor types: {executor_types}")
    print(f"Anytime type: {anytime_type}")
    print(f"Separate threads: {separate_threads}")
    print(f"Batch size: {batch_size}")

    
    plot_data(executor_types, anytime_type, separate_threads, batch_size)

if __name__ == "__main__":
    main()