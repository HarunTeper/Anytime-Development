import pandas as pd
import matplotlib.pyplot as plt
import sys
import os
import glob
import numpy as np
import argparse


def calculate_time_difference(df, start_timestamp, end_timestamp):
    """
    Calculate time difference between two timestamps in milliseconds

    Args:
        df: DataFrame containing timestamp columns
        start_timestamp: Column name of start timestamp
        end_timestamp: Column name of end timestamp

    Returns:
        numpy array with time differences in milliseconds
    """
    if start_timestamp not in df.columns or end_timestamp not in df.columns:
        print(
            f"Warning: Timestamp columns '{start_timestamp}' or '{end_timestamp}' not found in data")
        return np.zeros(len(df))

    # Calculate difference in nanoseconds and convert to milliseconds
    time_diff = (df[end_timestamp].to_numpy() -
                 df[start_timestamp].to_numpy()) / 1e6
    return time_diff


def plot_metrics_generic(x_values, metrics_dict, title, ylabel, output_path, figsize=(10, 6)):
    """
    Generic plotting function that can plot multiple metrics on the same chart

    Args:
        x_values: numpy array for x-axis values
        metrics_dict: dictionary mapping metric names to values
        title: plot title
        ylabel: y-axis label
        output_path: path to save the plot
        figsize: figure size as tuple (width, height)
    """
    plt.figure(figsize=figsize)

    for metric_name, metric_values in metrics_dict.items():
        plt.plot(x_values, metric_values, label=metric_name)

    plt.title(title)
    plt.xlabel('Sample Index')
    plt.ylabel(ylabel)
    plt.grid(True)
    if len(metrics_dict) > 1:
        plt.legend()
    plt.savefig(output_path)
    plt.close()


def combine_run_data(file_pattern):
    """
    Combine data from multiple runs into a single DataFrame

    Args:
        file_pattern: Pattern to match multiple run files

    Returns:
        Combined DataFrame or None if no files found
    """

    print(f"Combining data from multiple runs using pattern: {file_pattern}")

    all_files = glob.glob(file_pattern)
    if not all_files:
        return None

    all_dfs = []
    for file in all_files:
        try:
            df = pd.read_csv(file, comment='#')
            # Add run identifier from filename - Fix to handle .csv extension
            run_num = int(file.split('_run')[-1].split('.')[0])
            df['run'] = run_num
            all_dfs.append(df)
            print(f"Successfully read {file} with {len(df)} rows")
        except Exception as e:
            try:
                df = pd.read_csv(file)
                # Fix to handle .csv extension
                run_num = int(file.split('_run')[-1].split('.')[0])
                df['run'] = run_num
                all_dfs.append(df)
                print(f"Successfully read {file} without comment parameter")
            except Exception as e2:
                print(f"Could not read file {file}: {e2}")

    if not all_dfs:
        return None

    # Combine all DataFrames
    combined_df = pd.concat(all_dfs, ignore_index=True)
    print(f"Combined {len(all_dfs)} runs with total {len(combined_df)} rows")

    return combined_df


def plot_raw_timestamps(csv_files, output_dir):
    """Plot metrics from multiple raw timestamp CSV files"""

    # Combine all runs into a single DataFrame
    combined_pattern = csv_files.replace('_run1', '_run*')
    combined_df = combine_run_data(combined_pattern)

    if combined_df is None:
        print(f"No data to plot for pattern: {combined_pattern}")
        return

    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)

    # Extract base filename for plot titles
    base_name = os.path.basename(csv_files).replace('_run1.csv', '')

    # Convert index to numpy array for plotting
    x_values = np.array(combined_df.index)

    # Define timestamp pairs to calculate latencies
    time_diff_pairs = {
        'total_client_start': ('client_goal_start', 'server_receive'),
        'client_start': ('client_goal_start', 'client_send_start'),
        'client_send_receive': ('client_send_start', 'server_receive'),

        'total_server_start': ('server_receive', 'server_start'),
        'server_accept': ('server_receive', 'server_accept'),
        'server_start': ('server_accept', 'server_start'),

        'server_start_finish': ('server_start', 'server_send_result'),
        'server_start_cancel': ('server_start', 'server_cancel'),
        'client_start_cancel': ('client_send_start', 'client_send_cancel_start'),
        'client_start_receive': ('client_send_start', 'client_result'),

        'total_client_cancel': ('client_send_cancel_start', 'client_result'),
        'client_cancel_receive': ('client_send_cancel_start', 'server_cancel'),
        'client_cancel_result_send': ('server_cancel', 'server_send_result'),
        'client_cancel_result_receive': ('server_send_result', 'client_result'),

        'server_start_response': ('server_accept', 'client_goal_response'),
        'client_send_latency': ('client_send_start', 'client_send_end'),
        'client_cancel_latency': ('client_send_cancel_start', 'client_send_cancel_end'),
    }

    # Calculate all time differences
    time_diffs = {}
    for diff_name, (start_col, end_col) in time_diff_pairs.items():
        time_diffs[diff_name] = calculate_time_difference(
            combined_df, start_col, end_col)

    # Extract other metrics directly from the dataframe
    if 'batch_time_ns' in combined_df.columns:
        # Convert from ns to ms
        time_diffs['batch_time'] = combined_df['batch_time_ns'].to_numpy() / \
            1e6

    if 'iterations' in combined_df.columns:
        iterations = combined_df['iterations'].to_numpy()
        time_diffs['iterations'] = iterations

    if 'batch_size' in combined_df.columns:
        batch_size = combined_df['batch_size'].to_numpy()
        time_diffs['batch_size'] = batch_size

        # Calculate iterations per batch unit if both metrics are available
        if 'iterations' in combined_df.columns:
            time_diffs['iterations_per_batch'] = iterations / batch_size

    # Calculate efficiency if we have both iterations and batch time
    if 'iterations' in combined_df.columns and 'batch_time_ns' in combined_df.columns:
        # iterations per millisecond
        time_diffs['efficiency'] = iterations / \
            (combined_df['batch_time_ns'].to_numpy() / 1e6)

    # Add run information to metrics
    if 'run' in combined_df.columns:
        time_diffs['run'] = combined_df['run'].to_numpy()

    # 1. Plot all latency metrics in one chart
    latency_metrics = {k: v for k, v in time_diffs.items() if k in [
        'total_client_start', 'client_start', 'client_send_latency', 'client_send_receive',
        'total_server_start', 'server_accept', 'server_start',
        'server_start_finish', 'server_start_cancel', 'client_start_cancel', 'client_start_receive',
        'total_client_cancel', 'client_cancel_latency', 'client_cancel_receive', 'client_cancel_result_send',
        'client_cancel_result_receive', 'server_start_response'
    ]}

    plot_metrics_generic(
        x_values,
        latency_metrics,
        f'Latency Metrics - {base_name} (Combined Runs)',
        'Time (ms)',
        f'{output_dir}/{base_name}_latencies.png',
        figsize=(15, 10)
    )

    # 2. Group and plot related metrics together for better comparison
    # Group 1: Goal creation and sending
    send_metrics = {k: v for k, v in time_diffs.items() if k in [
        'total_client_start', 'client_start', 'client_send_receive'

    ]}
    plot_metrics_generic(
        x_values,
        send_metrics,
        f'Goal Sending Metrics - {base_name} (Combined Runs)',
        'Time (ms)',
        f'{output_dir}/{base_name}_send_metrics.png'
    )

    # Group 2: Server processing time
    server_metrics = {k: v for k, v in time_diffs.items() if k in [
        'server_accept', 'server_start', 'total_server_start'
    ]}
    plot_metrics_generic(
        x_values,
        server_metrics,
        f'Server Processing Metrics - {base_name} (Combined Runs)',
        'Time (ms)',
        f'{output_dir}/{base_name}_server_metrics.png'
    )

    # Group 3: Cancellation metrics
    cancel_metrics = {k: v for k, v in time_diffs.items() if k in [
        'total_client_cancel', 'client_cancel_receive', 'client_cancel_result_send',
        'client_cancel_result_receive'
    ]}
    plot_metrics_generic(
        x_values,
        cancel_metrics,
        f'Cancellation Metrics - {base_name} (Combined Runs)',
        'Time (ms)',
        f'{output_dir}/{base_name}_cancel_metrics.png'
    )

    # Group 4 - Processing times
    processing_metrics = {k: v for k, v in time_diffs.items() if k in [
        'server_start_finish', 'server_start_cancel', 'client_start_cancel', 'client_start_receive'
    ]}
    plot_metrics_generic(
        x_values,
        processing_metrics,
        f'Processing Times - {base_name} (Combined Runs)',
        'Time (ms)',
        f'{output_dir}/{base_name}_processing_metrics.png'
    )

    # Group 5 - Response time
    response_metrics = {k: v for k, v in time_diffs.items() if k in [
        'server_start_response', 'client_send_latency', 'client_cancel_latency'
    ]}

    plot_metrics_generic(
        x_values,
        response_metrics,
        f'Response Time Metrics - {base_name} (Combined Runs)',
        'Time (ms)',
        f'{output_dir}/{base_name}_response_metrics.png'
    )

    # 3. Plot individual metrics
    # Plot batch time
    if 'batch_time' in time_diffs:
        plot_metrics_generic(
            x_values,
            {'batch_time': time_diffs['batch_time']},
            f'Batch Processing Time - {base_name} (Combined Runs)',
            'Time (ms)',
            f'{output_dir}/{base_name}_batch_time.png'
        )

    # Plot iterations
    if 'iterations' in time_diffs:
        plot_metrics_generic(
            x_values,
            {'iterations': time_diffs['iterations']},
            f'Total Iterations - {base_name} (Combined Runs)',
            'Iterations',
            f'{output_dir}/{base_name}_iterations.png'
        )

    # Plot iterations per batch
    if 'iterations_per_batch' in time_diffs:
        plot_metrics_generic(
            x_values,
            {'iterations_per_batch': time_diffs['iterations_per_batch']},
            f'Iterations per Batch Unit - {base_name} (Combined Runs)',
            'Iterations/Batch Size',
            f'{output_dir}/{base_name}_iterations_per_batch.png'
        )

    # Plot efficiency (iterations per millisecond)
    if 'efficiency' in time_diffs:
        plot_metrics_generic(
            x_values,
            {'efficiency': time_diffs['efficiency']},
            f'Processing Efficiency - {base_name} (Combined Runs)',
            'Iterations per millisecond',
            f'{output_dir}/{base_name}_processing_efficiency.png'
        )


def plot_batch_size_comparison(threading_types, reactive_types, batch_sizes, num_runs, output_dir):
    """
    Plot comparisons of latency metrics across different batch sizes.

    Args:
        threading_types: List of threading types (single, multi)
        reactive_types: List of reactive types (reactive, proactive)
        batch_sizes: List of batch sizes
        num_runs: Number of runs per configuration
        output_dir: Directory to save output plots
    """
    print("Creating batch size comparison plots combining data from multiple runs...")

    # Metrics to compare across batch sizes
    metrics_to_compare = [
        'batch_time',
        'iterations',
        'efficiency',
        'iterations_per_batch'
    ]

    # Latency metrics to compare - expanded to include all latency metrics
    latency_metrics = [
        'total_client_start',
        'client_start',
        'client_send_receive',

        'total_server_start',
        'server_accept',
        'server_start',

        'server_start_finish',
        'server_start_cancel',
        'client_start_cancel',
        'client_start_receive',

        'total_client_cancel',
        'client_cancel_receive',
        'client_cancel_result_send',
        'client_cancel_result_receive',

        'server_start_response',
        'client_send_latency',
        'client_cancel_latency'
    ]

    # Dictionary to store all data by configuration and metric
    all_data = {}

    # Collect data for all configurations
    for threading in threading_types:
        for reactive in reactive_types:
            config = f"{threading}-{reactive}"
            all_data[config] = {}

            print(f"Collecting data for {config}...")

            # Collect data for each batch size
            for batch_size in batch_sizes:
                # Match all runs for this configuration
                pattern = f"results/monte_carlo/anytime_raw_timestamps_batch_{batch_size}_{reactive}_{threading}_run*.csv"
                matching_files = glob.glob(pattern)

                if not matching_files:
                    # Try the old timing data pattern as fallback
                    pattern = f"results/monte_carlo/anytime_timing_data_batch_{batch_size}_{reactive}_{threading}_run*.csv"
                    matching_files = glob.glob(pattern)

                if not matching_files:
                    print(
                        f"Warning: No data files found for batch_size={batch_size}, threading={threading}, reactive={reactive}")
                    continue

                print(
                    f"Found {len(matching_files)} files for batch_size={batch_size}")

                # Read and combine data from all runs
                dfs = []
                for file in matching_files:
                    try:
                        df = pd.read_csv(file, comment='#')
                        # Fix to handle .csv extension
                        run_num = int(file.split('_run')[-1].split('.')[0])
                        df['run'] = run_num
                        print(f"Read {len(df)} rows from {file}")
                        dfs.append(df)
                    except Exception as e:
                        try:
                            df = pd.read_csv(file)
                            # Fix to handle .csv extension
                            run_num = int(file.split('_run')[-1].split('.')[0])
                            df['run'] = run_num
                            dfs.append(df)
                        except Exception as e2:
                            print(f"Error reading file {file}: {e2}")

                if not dfs:
                    continue

                # Combine all runs for this configuration
                combined_df = pd.concat(dfs, ignore_index=True)
                print(
                    f"Combined {len(dfs)} runs with total {len(combined_df)} rows for batch_size={batch_size}")

                # Calculate metrics for this batch size across all runs
                metrics = {}

                # Complete time difference pairs for all latency metrics
                time_diff_pairs = {
                    'total_client_start': ('client_goal_start', 'server_receive'),
                    'client_start': ('client_goal_start', 'client_send_start'),
                    'client_send_receive': ('client_send_start', 'server_receive'),

                    'total_server_start': ('server_receive', 'server_start'),
                    'server_accept': ('server_receive', 'server_accept'),
                    'server_start': ('server_accept', 'server_start'),

                    'server_start_finish': ('server_start', 'server_send_result'),
                    'server_start_cancel': ('server_start', 'server_cancel'),
                    'client_start_cancel': ('client_send_start', 'client_send_cancel_start'),
                    'client_start_receive': ('client_send_start', 'client_result'),

                    'total_client_cancel': ('client_send_cancel_start', 'client_result'),
                    'client_cancel_receive': ('client_send_cancel_start', 'server_cancel'),
                    'client_cancel_result_send': ('server_cancel', 'server_send_result'),
                    'client_cancel_result_receive': ('server_send_result', 'client_result'),

                    'server_start_response': ('server_accept', 'client_goal_response'),
                    'client_send_latency': ('client_send_start', 'client_send_end'),
                    'client_cancel_latency': ('client_send_cancel_start', 'client_send_cancel_end'),
                }

                for diff_name, (start_col, end_col) in time_diff_pairs.items():
                    if start_col in combined_df.columns and end_col in combined_df.columns:
                        metrics[diff_name] = calculate_time_difference(
                            combined_df, start_col, end_col)

                # Extract additional metrics
                if 'batch_time_ns' in combined_df.columns:
                    metrics['batch_time'] = combined_df['batch_time_ns'].to_numpy() / \
                        1e6

                if 'iterations' in combined_df.columns:
                    metrics['iterations'] = combined_df['iterations'].to_numpy()

                if 'iterations' in combined_df.columns and 'batch_size' in combined_df.columns:
                    metrics['iterations_per_batch'] = combined_df['iterations'].to_numpy(
                    ) / combined_df['batch_size'].to_numpy()

                if 'iterations' in combined_df.columns and 'batch_time_ns' in combined_df.columns:
                    metrics['efficiency'] = combined_df['iterations'].to_numpy(
                    ) / (combined_df['batch_time_ns'].to_numpy() / 1e6)

                print(f"Calculated metrics for batch_size={batch_size}")

                # For each metric, store the batch size data
                for metric_name, metric_data in metrics.items():
                    if metric_name not in all_data[config]:
                        all_data[config][metric_name] = {}
                    all_data[config][metric_name][batch_size] = metric_data

    # Define colors for different configurations
    config_colors = {
        'single-reactive': 'blue',
        'single-proactive': 'green',
        'multi-reactive': 'red',
        'multi-proactive': 'purple'
    }

    def create_boxplots_for_metrics(metrics_list, is_latency_metric=False):
        """
        Helper function to create boxplots for a list of metrics

        Args:
            metrics_list: List of metrics to plot
            is_latency_metric: Whether these are latency metrics (affects y-axis label)
        """
        for metric_name in metrics_list:
            print(f"Creating boxplot for metric: {metric_name}")

            # Get all batch sizes that have data
            all_batch_sizes = sorted(set(bs for config in all_data.values()
                                         if metric_name in config
                                         for bs in config[metric_name].keys()))

            if not all_batch_sizes:
                print(f"No data available for metric {metric_name}")
                continue

            # Create figure with appropriate size
            fig, ax = plt.subplots(
                figsize=(max(12, len(all_batch_sizes) * 2), 8))

            # Track positions for boxplots and labels
            positions = []
            labels = []
            box_data = []
            box_colors = []

            # Space between groups of boxplots
            group_space = 2
            # Space between boxplots in a group
            box_space = 0.6

            pos = 0
            # For each batch size, plot boxplots for each configuration
            for i, batch_size in enumerate(all_batch_sizes):
                pos = i * (len(config_colors) + group_space) + 1

                for j, (config, metrics) in enumerate(sorted(all_data.items())):
                    if metric_name in metrics and batch_size in metrics[metric_name]:
                        data = metrics[metric_name][batch_size]
                        if len(data) > 0:
                            current_pos = pos + j * box_space
                            positions.append(current_pos)
                            box_data.append(data)
                            box_colors.append(
                                config_colors.get(config, 'gray'))
                            labels.append(f"{config}")

            # Create boxplots
            boxes = ax.boxplot(box_data, positions=positions, patch_artist=True,
                               widths=0.4, showfliers=False, notch=False)

            # Customize boxplot colors
            for box, color in zip(boxes['boxes'], box_colors):
                box.set(facecolor=color, alpha=0.6)
                box.set(edgecolor=color, linewidth=1.5)

            for whisker, color in zip(boxes['whiskers'], [c for c in box_colors for _ in range(2)]):
                whisker.set(color=color, linewidth=1.5)

            for cap, color in zip(boxes['caps'], [c for c in box_colors for _ in range(2)]):
                cap.set(color=color, linewidth=1.5)

            for median, color in zip(boxes['medians'], box_colors):
                median.set(color='black', linewidth=1.5)

            # Add lines connecting mean values across batch sizes
            # First, collect mean values by configuration and batch size
            config_means = {config: {} for config in all_data.keys()}
            config_stderrs = {config: {} for config in all_data.keys()}

            for i, batch_size in enumerate(all_batch_sizes):
                for config, metrics in all_data.items():
                    if metric_name in metrics and batch_size in metrics[metric_name]:
                        data = metrics[metric_name][batch_size]
                        if len(data) > 0:
                            config_means[config][batch_size] = np.mean(data)
                            # Standard error for error bars
                            config_stderrs[config][batch_size] = np.std(
                                data) / np.sqrt(len(data))

            # Now plot lines connecting the means with error bars
            for config, means in config_means.items():
                if len(means) > 1:  # Only plot if we have at least two points
                    x_values = []
                    y_values = []
                    errors = []
                    for batch_size in sorted(means.keys()):
                        # Find the x-position for this batch size
                        batch_idx = all_batch_sizes.index(batch_size)
                        config_idx = sorted(all_data.keys()).index(config)
                        x_pos = batch_idx * \
                            (len(config_colors) + group_space) + \
                            1 + config_idx * box_space

                        x_values.append(x_pos)
                        y_values.append(means[batch_size])
                        errors.append(config_stderrs[config][batch_size])

                    # Plot the line connecting means with the same color as the boxes
                    plt.errorbar(x_values, y_values, yerr=errors,
                                 fmt='-o', color=config_colors.get(config, 'gray'),
                                 linewidth=2, alpha=0.7,
                                 capsize=4, elinewidth=1.5)

            # Set x-ticks at the middle of each group
            group_positions = [i * (len(config_colors) + group_space) + 1 + (len(config_colors) * box_space) / 2
                               for i in range(len(all_batch_sizes))]

            # Add vertical lines between batch size groups
            if len(all_batch_sizes) > 1:
                for i in range(len(all_batch_sizes) - 1):
                    # Calculate position between current batch size group and next one
                    current_group_end = group_positions[i] + \
                        (len(config_colors) * box_space) / 2
                    next_group_start = group_positions[i+1] - \
                        (len(config_colors) * box_space) / 2
                    line_pos = (current_group_end + next_group_start) / 2

                    # Draw a vertical line
                    plt.axvline(x=line_pos, color='gray',
                                linestyle='--', alpha=0.5)

            ax.set_xticks(group_positions)
            ax.set_xticklabels([str(bs) for bs in all_batch_sizes])

            # Add batch size labels
            plt.xlabel('Batch Size')

            # Set y-axis label based on metric type
            if is_latency_metric:
                plt.ylabel('Time (ms)')
            else:
                plt.ylabel(f'{metric_name.replace("_", " ").title()}')

            plt.title(
                f'{metric_name.replace("_", " ").title()} vs Batch Size - All Configurations (Combined Runs)')
            plt.grid(True, axis='y')

            # Add legend for configurations
            legend_handles = [plt.Rectangle((0, 0), 1, 1, color=color, alpha=0.6)
                              for color in config_colors.values()]
            legend_labels = list(config_colors.keys())
            plt.legend(legend_handles, legend_labels, loc='best')

            # Adjust layout for better display
            plt.tight_layout()

            # Save the plot
            filename = f'{output_dir}/{metric_name}_boxplot_batch_size_combined.png'
            plt.savefig(filename)
            plt.close()
            print(f"Saved boxplot to {filename}")

    # Plot regular performance metrics
    create_boxplots_for_metrics(metrics_to_compare, is_latency_metric=False)

    # Plot latency metrics
    create_boxplots_for_metrics(latency_metrics, is_latency_metric=True)


def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(
        description='Plot metrics from raw timestamp files')
    parser.add_argument('--threading', nargs='+',
                        help='Threading types (single, multi)', required=True)
    parser.add_argument('--reactive', nargs='+',
                        help='Reactive types (reactive, proactive)', required=True)
    parser.add_argument('--batch-sizes', nargs='+', type=int,
                        help='Batch sizes', required=True)
    parser.add_argument('--runs', type=int, default=1,
                        help='Number of runs per configuration')
    parser.add_argument('--results-dir', type=str, default='results/monte_carlo',
                        help='Directory containing result files')
    parser.add_argument('--output-dir', type=str, default='plots/monte_carlo',
                        help='Directory to save output plots')

    args = parser.parse_args()

    print(f"Thread types: {args.threading}")
    print(f"Reactive types: {args.reactive}")
    print(f"Batch sizes: {args.batch_sizes}")
    print(f"Number of runs: {args.runs}")
    print(f"Results directory: {args.results_dir}")
    print(f"Output directory: {args.output_dir}")

    output_dir = args.output_dir
    os.makedirs(output_dir, exist_ok=True)

    # Process each configuration
    for threading in args.threading:
        for reactive in args.reactive:
            for batch_size in args.batch_sizes:
                print(
                    f"Processing configuration: threading={threading}, reactive={reactive}, batch_size={batch_size}")

                # Use first run file as reference, but will combine all runs
                file_pattern = f"{args.results_dir}/anytime_raw_timestamps_batch_{batch_size}_{reactive}_{threading}_run1.csv"

                # Plot combined data from all runs
                plot_raw_timestamps(file_pattern, output_dir)

    # After plotting individual files, create batch size comparison plots
    plot_batch_size_comparison(
        args.threading, args.reactive, args.batch_sizes, args.runs, output_dir)

    print("Plotting completed.")


if __name__ == "__main__":
    main()
