import os
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys

def plot_data(threading, reactive, separate, batch_size):
    threading_types = ["False", "True"] if threading == "Both" else [threading]
    anytime_reactives = ["False", "True"] if reactive == "Both" else [reactive]
    separate_threads = ["False", "True"] if separate == "Both" else [separate]

    data = {}

    for threading in threading_types:
        for reactive in anytime_reactives:
            for separate in separate_threads:
                config_name = f"anytime_{threading}_{reactive}_{separate}_{batch_size}"
                
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
                
                for run in range(1, 4):
                    file_path = f"/home/vscode/workspace/results/{config_name}_run{run}.log"
                    print(f"Checking file {file_path}")
                    if os.path.exists(file_path):
                        with open(file_path, 'r') as file:
                            lines = file.readlines()
                            last_iteration_line = None
                            for line in lines:
                                if "Number of iterations: mean=" in line:
                                    last_iteration_line = line
                            if last_iteration_line:
                                lines = lines[:lines.index(last_iteration_line)+1]
                            lines = lines[-7:]
                            
                            for line in lines:
                                if not line.strip():
                                    continue
                                
                                name = line.split(":")[1].strip()
                                values = line.split(":")[2].split()
                                values = [value.replace(",", "") for value in values if "=" in value]
                                
                                mean = float(values[0].split("=")[1])
                                stdev = float(values[1].split("=")[1])
                                p99 = float(values[2].split("=")[1])
                                p999 = float(values[3].split("=")[1])
                                max_value = float(values[4].split("=")[1])
                                
                                if name not in value_dict:
                                    value_dict[name] = {
                                        "mean": [],
                                        "stdev": [],
                                        "p99": [],
                                        "p999": [],
                                        "max": []
                                    }
                                
                                value_dict[name]["mean"].append(mean)
                                value_dict[name]["stdev"].append(stdev)
                                value_dict[name]["p99"].append(p99)
                                value_dict[name]["p999"].append(p999)
                                value_dict[name]["max"].append(max_value)
                                
                                # print p99 value of value_dict
                                print(f"Configuration99 {key_name} - {name}: {value_dict[name]}")
                
                for name in value_dict:
                    value_dict[name] = {
                        "mean": np.mean(value_dict[name]["mean"]),
                        "stdev": np.mean(value_dict[name]["stdev"]),
                        "p99": np.mean(value_dict[name]["p99"]),
                        "p999": np.mean(value_dict[name]["p999"]),
                        "max": np.mean(value_dict[name]["max"])
                    }
                    # print(f"Configuration {key_name} - {name}: {value_dict[name]}")
                
                data[key_name] = value_dict
    
    for config in data:
        for interval in data[config]:
            if interval != 'Number of iterations':
                data[config][interval]["mean"] = data[config][interval]["mean"]
                data[config][interval]["stdev"] = data[config][interval]["stdev"]
                data[config][interval]["p99"] = data[config][interval]["p99"]
                data[config][interval]["p999"] = data[config][interval]["p999"]
                data[config][interval]["max"] = data[config][interval]["max"]
                
    intervals = list(data[next(iter(data))].keys())
    configs = list(data.keys())

    for interval in intervals:
        interval_df = pd.DataFrame(columns=["Configuration", "Mean", "Stdev", "P99", "P999", "Max"])
        rows = []
        for config in data:
            rows.append({
                "Configuration": config,
                "Mean": data[config][interval]["mean"],
                "Stdev": data[config][interval]["stdev"],
                "P99": data[config][interval]["p99"],
                "P999": data[config][interval]["p999"],
                "Max": data[config][interval]["max"]
            })
        if rows:
            interval_df = pd.concat([interval_df, pd.DataFrame(rows)], ignore_index=True)
        
        interval_df.to_csv(f"/home/vscode/workspace/plots/evaluation_data_{interval}_{batch_size}.csv", index=False)
    

    colors = {
        "mean": "skyblue",
        "stdev": "orange",
        "p99": "lightgreen",
        "p999": "purple",
        "max": "salmon"
    }

    fig, axs = plt.subplots(len(intervals), 1, figsize=(12, 4 * len(intervals)))
    fig.tight_layout(pad=10.0)

    for i, interval in enumerate(intervals):
        ax = axs[i] if len(intervals) > 1 else axs
        
        means = [data[config][interval]["mean"] for config in configs]
        stdevs = [data[config][interval]["stdev"] for config in configs]
        p99s = [data[config][interval]["p99"] for config in configs]
        p999s = [data[config][interval]["p999"] for config in configs]
        maxs = [data[config][interval]["max"] for config in configs]

        bar_width = 0.2
        indices = np.arange(len(configs))
        
        max_value = max(maxs)
        ax.set_ylim(0, max_value + 0.1)
        
        ax.bar(indices - 1.5 * bar_width, means, bar_width, label="Mean", color=colors["mean"])
        ax.bar(indices - 0.5 * bar_width, stdevs, bar_width, label="Std Dev", color=colors["stdev"])
        ax.bar(indices + 0.5 * bar_width, p99s, bar_width, label="P99", color=colors["p99"])
        ax.bar(indices + 1.5 * bar_width, p999s, bar_width, label="P999", color=colors["p999"])
        ax.bar(indices + 2.5 * bar_width, maxs, bar_width, label="Max", color=colors["max"])

        ax.set_title(interval)
        ax.set_xticks(indices)
        ax.set_xticklabels(configs, rotation=45, ha="right")
        ax.set_ylabel("Value")
        ax.legend()

    plt.savefig("/home/vscode/workspace/plots/evaluation_plot_"+str(batch_size)+".png")
    
    configurations = list(data.keys())
    intervals = [key for key in data[configurations[0]].keys()]
    
    for config in configurations:
        for interval in intervals:
            if data[config][interval]["max"] > data[config][interval]["p999"]:
                data[config][interval]["max"] = data[config][interval]["p999"]

    interval_data = {interval: [] for interval in intervals}
    for interval in intervals:
        for config in configurations:
            stats = data[config][interval]
            interval_data[interval].append([stats['mean'], stats['p99'], stats['p999'], stats['max']])

    fig, ax = plt.subplots(len(intervals), 1, figsize=(15, 3 * len(intervals)), sharex=True)
    fig.suptitle('Boxplots for Each Interval by Configuration', fontsize=16)

    for idx, interval in enumerate(intervals):
        ax[idx].boxplot(interval_data[interval], tick_labels=configurations, patch_artist=True)
        ax[idx].set_title(interval)
        ax[idx].set_ylabel('Latency (ms)')
        ax[idx].set_ylim(0, max([max(data) for data in interval_data[interval]]) + 0.1)
        
    plt.xticks(rotation=45, ha="right")
    plt.xlabel('Configuration')
    plt.tight_layout(rect=[0, 0, 1, 0.95])

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
