#!/usr/bin/env python3
"""
Monte Carlo Experiment Evaluation Script

This script parses LTTng traces from Monte Carlo experiments and generates:
- Metrics: iterations per block, blocks completed, time per block, cancellation delay
- Plots: block size comparisons, mode comparisons, threading comparisons
- CSV/JSON exports of results
"""

import sys
import json
import subprocess
from pathlib import Path
from collections import defaultdict
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from datetime import datetime
from concurrent.futures import ProcessPoolExecutor

# Configuration
SCRIPT_DIR = Path(__file__).resolve().parent
EXPERIMENT_DIR = SCRIPT_DIR
TRACE_DIR = EXPERIMENT_DIR / "traces"
RESULTS_DIR = EXPERIMENT_DIR / "results"
PLOTS_DIR = RESULTS_DIR / "plots"

# Plot configuration
PLOT_WIDTH = 12
PLOT_HEIGHT = 8
PLOT_HEIGHT_SMALL = 8
PLOT_DPI = 300
FONT_SIZE_TITLE = 30
FONT_SIZE_LABEL = 30
FONT_SIZE_LEGEND = 30
FONT_SIZE_TICK_LABELS = 30
FONT_SIZE_OFFSET = 30  # Size for scientific notation offset (e.g., "1e6")
LEGEND_SIZE = 30
MARKER_SIZE = 12
CAPSIZE = 5
LINE_WIDTH = 2

# Create output directories
RESULTS_DIR.mkdir(exist_ok=True)
PLOTS_DIR.mkdir(exist_ok=True)


class TraceEvent:
    """Represents a single trace event"""

    def __init__(self, timestamp, event_name, fields):
        self.timestamp = timestamp
        self.event_name = event_name
        self.fields = fields

    def __repr__(self):
        return f"TraceEvent({self.timestamp}, {self.event_name})"


def parse_trace_directory(trace_dir):
    """
    Parse a single trace directory using babeltrace with filtering
    """
    print(f"  Parsing trace: {trace_dir.name}")

    try:
        result = subprocess.run(
            ['babeltrace', str(trace_dir)],
            capture_output=True,
            text=True,
            check=True
        )
    except subprocess.CalledProcessError as e:
        print(f"    Error running babeltrace: {e}")
        return []
    except FileNotFoundError:
            print("    Error: babeltrace not found. Please install lttng-tools.")
            return []

    # Pre-filter lines to only process anytime events
    anytime_lines = [line for line in result.stdout.split(
        '\n') if 'anytime:' in line]

    events = []
    for line in anytime_lines:
        if not line.strip() or not 'anytime:' in line:
            continue

        # Parse the trace line
        # Format: [timestamp] hostname anytime:event_name: { fields }
        try:
            # Extract timestamp (in brackets)
            ts_start = line.find('[')
            ts_end = line.find(']')
            if ts_start == -1 or ts_end == -1:
                continue

            timestamp_str = line[ts_start+1:ts_end]
            # Convert HH:MM:SS.nanoseconds to nanoseconds
            ts_parts = timestamp_str.split(':')
            timestamp = (int(ts_parts[0]) * 3600 + int(ts_parts[1]) * 60
                         + float(ts_parts[2])) * 1e9

            # Extract event name (format: "anytime:event_name:")
            event_start = line.find('anytime:')
            if event_start == -1:
                continue
            event_part = line[event_start:]
            # Find the colon AFTER "anytime:"
            event_end = event_part.find(':', len('anytime:'))
            if event_end == -1:
                continue
            event_name = event_part[:event_end]

            # Extract fields (handle nested braces for context)
            fields = {}
            # Find all { } groups
            brace_groups = []
            brace_level = 0
            current_start = -1
            for i, char in enumerate(line):
                if char == '{':
                    if brace_level == 0:
                        current_start = i
                    brace_level += 1
                elif char == '}':
                    brace_level -= 1
                    if brace_level == 0 and current_start != -1:
                        brace_groups.append(line[current_start+1:i])
                        current_start = -1

            # Parse the last brace group (contains the actual event fields)
            if len(brace_groups) > 0:
                fields_str = brace_groups[-1]
                # Simple field parsing (key = value pairs)
                for field_pair in fields_str.split(','):
                    if '=' in field_pair:
                        key, value = field_pair.split('=', 1)
                        key = key.strip()
                        value = value.strip()
                        fields[key] = value

            events.append(TraceEvent(timestamp, event_name, fields))

        except Exception as e:
            # Skip malformed lines
            continue

    print(f"    Found {len(events)} anytime events")
    return events


def extract_metrics_from_events(events, config_name):
    """
    Extract metrics from a list of trace events

    Returns:
        Dictionary with metrics
    """
    # Parse block_size from config_name (format: block_<size>_<mode>_<threading>_run<N>)
    config_parts = config_name.split('_')
    block_size = int(config_parts[1])  # Extract block size from config name

    metrics = {
        'config': config_name,
        'block_size': block_size,
        'total_blocks': 0,
        'total_iterations': 0,  # Will be calculated as total_blocks * block_size
        'block_times': [],  # Time per block in ms
        'iterations_per_block': [],  # Will be set to block_size
        'server_cancel_response_delays': [],  # Time from cancel to deactivate in ms
        'compute_times': [],  # Time spent in compute
        'feedback_times': [],  # Time spent sending feedback
        'result_times': [],  # Time spent calculating results
        # New latency metrics
        # Time from goal sent to goal finished (ms)
        'goal_to_finish_latencies': [],
        # Time from goal sent to cancel sent (ms)
        'goal_to_cancel_latencies': [],
        # Time from cancel sent to goal finished (ms)
        'cancel_to_finish_latencies': [],
    }

    # Track state for computing metrics
    current_compute_start = None
    current_feedback_start = None
    current_result_start = None
    cancel_request_time = None
    # New: Track goal and cancel timestamps
    goal_sent_time = None
    cancel_sent_time = None

    for event in events:
        event_name = event.event_name

        # Track compute blocks
        if event_name == 'anytime:anytime_compute_entry':
            current_compute_start = event.timestamp
            metrics['total_blocks'] += 1

        elif event_name == 'anytime:anytime_compute_exit':
            if current_compute_start is not None:
                block_time_ms = (event.timestamp - current_compute_start) / 1e6
                metrics['block_times'].append(block_time_ms)
                metrics['compute_times'].append(block_time_ms)
                current_compute_start = None

        # Note: anytime_compute_iteration is no longer tracked
        # Iterations are calculated as: total_blocks * block_size

        # Track feedback timing
        elif event_name == 'anytime:anytime_send_feedback_entry':
            current_feedback_start = event.timestamp

        elif event_name == 'anytime:anytime_send_feedback_exit':
            if current_feedback_start is not None:
                feedback_time_ms = (
                    event.timestamp - current_feedback_start) / 1e6
                metrics['feedback_times'].append(feedback_time_ms)
                current_feedback_start = None

        # Track result calculation timing
        elif event_name == 'anytime:anytime_calculate_result_entry':
            current_result_start = event.timestamp

        elif event_name == 'anytime:anytime_calculate_result_exit':
            if current_result_start is not None:
                result_time_ms = (event.timestamp - current_result_start) / 1e6
                metrics['result_times'].append(result_time_ms)
                current_result_start = None

        # Track cancellation delay
        elif event_name == 'anytime:anytime_server_handle_cancel':
            cancel_request_time = event.timestamp

        elif event_name == 'anytime:anytime_base_deactivate':
            if cancel_request_time is not None:
                cancellation_delay_ms = (
                    event.timestamp - cancel_request_time) / 1e6
                metrics['server_cancel_response_delays'].append(cancellation_delay_ms)
                cancel_request_time = None

        # New: Track goal sent timestamp
        elif event_name == 'anytime:anytime_client_goal_sent':
            goal_sent_time = event.timestamp

        # New: Track cancel sent timestamp
        elif event_name == 'anytime:anytime_client_cancel_sent':
            cancel_sent_time = event.timestamp
            # Calculate goal to cancel latency
            if goal_sent_time is not None:
                latency_ms = (cancel_sent_time - goal_sent_time) / 1e6
                metrics['goal_to_cancel_latencies'].append(latency_ms)

        # New: Track goal finished timestamp
        elif event_name == 'anytime:anytime_client_goal_finished':
            goal_finished_time = event.timestamp

            # Calculate goal to finish latency
            if goal_sent_time is not None:
                latency_ms = (goal_finished_time - goal_sent_time) / 1e6
                metrics['goal_to_finish_latencies'].append(latency_ms)

            # Calculate cancel to finish latency
            if cancel_sent_time is not None:
                latency_ms = (goal_finished_time - cancel_sent_time) / 1e6
                metrics['cancel_to_finish_latencies'].append(latency_ms)

            # Reset for next goal
            goal_sent_time = None
            cancel_sent_time = None

    # Compute summary statistics
    if metrics['block_times']:
        metrics['avg_time_per_block'] = np.mean(metrics['block_times'])
        metrics['std_time_per_block'] = np.std(metrics['block_times'])
    else:
        metrics['avg_time_per_block'] = 0
        metrics['std_time_per_block'] = 0

    # Calculate iterations from block_size * total_blocks
    metrics['total_iterations'] = metrics['total_blocks'] * block_size
    metrics['avg_iterations_per_block'] = block_size
    # No variation since it's the configured block_size
    metrics['std_iterations_per_block'] = 0

    if metrics['server_cancel_response_delays']:
        metrics['avg_server_cancel_response'] = np.mean(
            metrics['server_cancel_response_delays'])
        metrics['std_server_cancel_response'] = np.std(
            metrics['server_cancel_response_delays'])
    else:
        metrics['avg_server_cancel_response'] = 0
        metrics['std_server_cancel_response'] = 0

    # New: Compute latency statistics
    if metrics['goal_to_finish_latencies']:
        metrics['avg_goal_to_finish_latency'] = np.mean(
            metrics['goal_to_finish_latencies'])
        metrics['std_goal_to_finish_latency'] = np.std(
            metrics['goal_to_finish_latencies'])
    else:
        metrics['avg_goal_to_finish_latency'] = 0
        metrics['std_goal_to_finish_latency'] = 0

    if metrics['goal_to_cancel_latencies']:
        metrics['avg_goal_to_cancel_latency'] = np.mean(
            metrics['goal_to_cancel_latencies'])
        metrics['std_goal_to_cancel_latency'] = np.std(
            metrics['goal_to_cancel_latencies'])
    else:
        metrics['avg_goal_to_cancel_latency'] = 0
        metrics['std_goal_to_cancel_latency'] = 0

    if metrics['cancel_to_finish_latencies']:
        metrics['avg_cancel_to_finish_latency'] = np.mean(
            metrics['cancel_to_finish_latencies'])
        metrics['std_cancel_to_finish_latency'] = np.std(
            metrics['cancel_to_finish_latencies'])
    else:
        metrics['avg_cancel_to_finish_latency'] = 0
        metrics['std_cancel_to_finish_latency'] = 0

    return metrics


def aggregate_runs(all_metrics):
    """
    Aggregate metrics from multiple runs of the same configuration

    Returns:
        Dictionary mapping config_name to aggregated metrics
    """
    # Group by base config name (without _runN suffix)
    config_groups = defaultdict(list)
    for metrics in all_metrics:
        # Remove _runN suffix
        base_config = metrics['config'].rsplit('_run', 1)[0]
        config_groups[base_config].append(metrics)

    # Aggregate each group
    aggregated = {}
    for base_config, runs in config_groups.items():
        agg = {
            'config': base_config,
            'num_runs': len(runs),
            'total_blocks': np.mean([r['total_blocks'] for r in runs]),
            'total_iterations': np.mean([r['total_iterations'] for r in runs]),
            'avg_time_per_block': np.mean([r['avg_time_per_block'] for r in runs]),
            'std_time_per_block': np.mean([r['std_time_per_block'] for r in runs]),
            'avg_iterations_per_block': np.mean([r['avg_iterations_per_block'] for r in runs]),
            'std_iterations_per_block': np.mean([r['std_iterations_per_block'] for r in runs]),
            'avg_server_cancel_response': np.mean([r['avg_server_cancel_response'] for r in runs if r['avg_server_cancel_response'] > 0]) if any(r['avg_server_cancel_response'] > 0 for r in runs) else 0,
            'std_server_cancel_response': np.mean([r['std_server_cancel_response'] for r in runs if r['std_server_cancel_response'] > 0]) if any(r['std_server_cancel_response'] > 0 for r in runs) else 0,
            # New latency metrics
            'avg_goal_to_finish_latency': np.mean([r['avg_goal_to_finish_latency'] for r in runs if r['avg_goal_to_finish_latency'] > 0]) if any(r['avg_goal_to_finish_latency'] > 0 for r in runs) else 0,
            'std_goal_to_finish_latency': np.mean([r['std_goal_to_finish_latency'] for r in runs if r['std_goal_to_finish_latency'] > 0]) if any(r['std_goal_to_finish_latency'] > 0 for r in runs) else 0,
            'avg_goal_to_cancel_latency': np.mean([r['avg_goal_to_cancel_latency'] for r in runs if r['avg_goal_to_cancel_latency'] > 0]) if any(r['avg_goal_to_cancel_latency'] > 0 for r in runs) else 0,
            'std_goal_to_cancel_latency': np.mean([r['std_goal_to_cancel_latency'] for r in runs if r['std_goal_to_cancel_latency'] > 0]) if any(r['std_goal_to_cancel_latency'] > 0 for r in runs) else 0,
            'avg_cancel_to_finish_latency': np.mean([r['avg_cancel_to_finish_latency'] for r in runs if r['avg_cancel_to_finish_latency'] > 0]) if any(r['avg_cancel_to_finish_latency'] > 0 for r in runs) else 0,
            'std_cancel_to_finish_latency': np.mean([r['std_cancel_to_finish_latency'] for r in runs if r['std_cancel_to_finish_latency'] > 0]) if any(r['std_cancel_to_finish_latency'] > 0 for r in runs) else 0,
        }
        aggregated[base_config] = agg

    return aggregated


def parse_config_name(config_name):
    """Parse configuration name into components"""
    # Format: block_<size>_<mode>_<threading>
    # or test_block_<size>_<mode>_<threading> for test runs
    parts = config_name.split('_')

    # Skip 'test' prefix if present
    offset = 1 if parts[0] == 'test' else 0

    return {
        'block_size': int(parts[1 + offset]),
        'mode': parts[2 + offset],
        'threading': parts[3 + offset]
    }


def generate_plots(aggregated_metrics):
    """Generate all plots from aggregated metrics"""

    print("\nGenerating plots...")

    # Convert to DataFrame for easier plotting
    df = pd.DataFrame([
        {**parse_config_name(config), **metrics}
        for config, metrics in aggregated_metrics.items()
    ])

    # Set plot style
    try:
        plt.style.use('seaborn-v0_8-darkgrid')
    except OSError:
        plt.style.use('seaborn-darkgrid')

    # Plot 1: Block Size vs Average Iterations per Block
    print("  - Block size vs iterations per block")
    fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT))

    # Get all unique block sizes present in the data
    all_block_sizes = sorted(df['block_size'].unique())
    x = np.arange(len(all_block_sizes))
    width = 0.2

    for i, mode in enumerate(['reactive', 'proactive']):
        for j, threading in enumerate(['single', 'multi']):
            data = df[(df['mode'] == mode) & (df['threading'] == threading)]
            data = data.sort_values('block_size')

            if data.empty:
                continue

            # Create aligned arrays for x positions and y values
            y_values = []
            yerr_values = []
            x_positions = []
            for block_size in all_block_sizes:
                block_data = data[data['block_size'] == block_size]
                if not block_data.empty:
                    y_values.append(
                        block_data['avg_iterations_per_block'].iloc[0])
                    yerr_values.append(
                        block_data['std_iterations_per_block'].iloc[0])
                    x_positions.append(all_block_sizes.index(block_size))

            if y_values:
                offset = (i * 2 + j - 1.5) * width
                ax.bar(np.array(x_positions) + offset, y_values, width,
                       label=f'{mode}-{threading}', yerr=yerr_values, capsize=CAPSIZE)

    ax.set_xlabel('Block Size', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Average Iterations per Block', fontsize=FONT_SIZE_LABEL)
    # ax.set_title('Block Size vs Average Iterations per Block', fontsize=FONT_SIZE_TITLE)
    ax.set_xticks(x)
    ax.set_xticklabels(all_block_sizes, fontsize=FONT_SIZE_TICK_LABELS)
    ax.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
    ax.yaxis.get_offset_text().set_fontsize(FONT_SIZE_OFFSET)
    # ax.legend(fontsize=LEGEND_SIZE)  # Legend removed
    ax.grid(True, axis='y')

    plt.tight_layout(pad=0)
    plt.savefig(PLOTS_DIR / 'block_size_vs_iterations.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()

    # Plot 2: Block Size vs Time per Block
    print("  - Block size vs time per block")
    fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT))

    # Get all unique block sizes present in the data
    all_block_sizes = sorted(df['block_size'].unique())
    x = np.arange(len(all_block_sizes))
    width = 0.2

    for i, mode in enumerate(['reactive', 'proactive']):
        for j, threading in enumerate(['single', 'multi']):
            data = df[(df['mode'] == mode) & (df['threading'] == threading)]
            data = data.sort_values('block_size')

            if data.empty:
                continue

            # Create aligned arrays for x positions and y values
            y_values = []
            yerr_values = []
            x_positions = []
            for block_size in all_block_sizes:
                block_data = data[data['block_size'] == block_size]
                if not block_data.empty:
                    y_values.append(block_data['avg_time_per_block'].iloc[0])
                    yerr_values.append(
                        block_data['std_time_per_block'].iloc[0])
                    x_positions.append(all_block_sizes.index(block_size))

            if y_values:
                offset = (i * 2 + j - 1.5) * width
                ax.bar(np.array(x_positions) + offset, y_values, width,
                       label=f'{mode}-{threading}', yerr=yerr_values, capsize=CAPSIZE)

    ax.set_xlabel('Block Size', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Average Time per Block (ms)', fontsize=FONT_SIZE_LABEL)
    # ax.set_title('Block Size vs Average Time per Block', fontsize=FONT_SIZE_TITLE)
    ax.set_xticks(x)
    ax.set_xticklabels(all_block_sizes, fontsize=FONT_SIZE_TICK_LABELS)
    ax.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
    ax.yaxis.get_offset_text().set_fontsize(FONT_SIZE_OFFSET)
    # ax.legend(fontsize=LEGEND_SIZE)  # Legend removed
    ax.grid(True, axis='y')

    plt.tight_layout(pad=0)
    plt.savefig(PLOTS_DIR / 'block_size_vs_time.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()

    # Plot 3: Server Cancel Response Delay Comparison
    print("  - Server cancel response delay comparison")
    fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT_SMALL))

    # Get all unique block sizes present in the data
    all_block_sizes = sorted(df['block_size'].unique())
    x = np.arange(len(all_block_sizes))
    width = 0.2

    for i, mode in enumerate(['reactive', 'proactive']):
        for j, threading in enumerate(['single', 'multi']):
            data = df[(df['mode'] == mode) & (df['threading'] == threading)]

            if data.empty:
                continue

            data = data.sort_values('block_size')

            # Create aligned arrays for x positions and y values
            y_values = []
            x_positions = []
            for block_size in all_block_sizes:
                block_data = data[data['block_size'] == block_size]
                if not block_data.empty:
                    y_values.append(
                        block_data['avg_server_cancel_response'].iloc[0])
                    x_positions.append(all_block_sizes.index(block_size))

            if y_values:
                offset = (i * 2 + j - 1.5) * width
                ax.bar(np.array(x_positions) + offset, y_values, width,
                       label=f'{mode}-{threading}')

    ax.set_xlabel('Block Size', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Server Cancel Response Delay (ms)', fontsize=FONT_SIZE_LABEL)
    # ax.set_title('Server Cancel Response Delay Comparison', fontsize=FONT_SIZE_TITLE)
    ax.set_xticks(x)
    ax.set_xticklabels(all_block_sizes, fontsize=FONT_SIZE_TICK_LABELS)
    ax.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
    ax.yaxis.get_offset_text().set_fontsize(FONT_SIZE_OFFSET)
    # ax.legend(fontsize=LEGEND_SIZE)  # Legend removed
    ax.grid(True, axis='y')

    plt.tight_layout(pad=0)
    plt.savefig(PLOTS_DIR / 'server_cancel_response.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()

    # Plot 4: Threading Comparison (removed - redundant with plots 1 and 2)
    # This plot is no longer needed since all configs are now on single plots

    # Plot 4: Total Throughput (iterations/second)
    print("  - Throughput analysis")
    fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT))

    # Calculate throughput: iterations per block / (time per block / 1000)
    df['throughput'] = df['avg_iterations_per_block'] / \
        (df['avg_time_per_block'] / 1000.0)

    # Get all unique block sizes present in the data
    all_block_sizes = sorted(df['block_size'].unique())
    x = np.arange(len(all_block_sizes))
    width = 0.2

    for i, mode in enumerate(['reactive', 'proactive']):
        for j, threading in enumerate(['single', 'multi']):
            data = df[(df['mode'] == mode) & (df['threading'] == threading)]
            data = data.sort_values('block_size')

            if data.empty:
                continue

            # Create aligned arrays for x positions and y values
            y_values = []
            x_positions = []
            for block_size in all_block_sizes:
                block_data = data[data['block_size'] == block_size]
                if not block_data.empty:
                    y_values.append(block_data['throughput'].iloc[0])
                    x_positions.append(all_block_sizes.index(block_size))

            if y_values:
                offset = (i * 2 + j - 1.5) * width
                ax.bar(np.array(x_positions) + offset, y_values, width,
                       label=f'{mode}-{threading}')

    ax.set_xlabel('Block Size', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Throughput (iterations/second)', fontsize=FONT_SIZE_LABEL)
    # ax.set_title('Block Size vs Throughput', fontsize=FONT_SIZE_TITLE)
    ax.set_xticks(x)
    ax.set_xticklabels(all_block_sizes, fontsize=FONT_SIZE_TICK_LABELS)
    ax.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
    ax.yaxis.get_offset_text().set_fontsize(FONT_SIZE_OFFSET)
    # ax.legend(fontsize=LEGEND_SIZE)  # Legend removed
    ax.grid(True, axis='y')

    plt.tight_layout(pad=0)
    plt.savefig(PLOTS_DIR / 'throughput.pdf', dpi=PLOT_DPI,
                bbox_inches='tight', pad_inches=0)
    plt.close()

    # Plot 5: End-to-End Latency (Goal to Finish)
    print("  - Goal to finish latency")
    fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT))

    # Get all unique block sizes present in the data
    all_block_sizes = sorted(df['block_size'].unique())
    x = np.arange(len(all_block_sizes))
    width = 0.2

    for i, mode in enumerate(['reactive', 'proactive']):
        for j, threading in enumerate(['single', 'multi']):
            data = df[(df['mode'] == mode) & (df['threading'] == threading)]
            data = data.sort_values('block_size')

            if data.empty:
                continue

            # Create aligned arrays for x positions and y values
            y_values = []
            yerr_values = []
            x_positions = []
            for block_size in all_block_sizes:
                block_data = data[data['block_size'] == block_size]
                if not block_data.empty:
                    y_values.append(
                        block_data['avg_goal_to_finish_latency'].iloc[0])
                    yerr_values.append(
                        block_data['std_goal_to_finish_latency'].iloc[0])
                    x_positions.append(all_block_sizes.index(block_size))

            if y_values:
                offset = (i * 2 + j - 1.5) * width
                ax.bar(np.array(x_positions) + offset, y_values, width,
                       label=f'{mode}-{threading}', yerr=yerr_values, capsize=CAPSIZE)

    ax.set_xlabel('Block Size', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Average Goal-to-Finish Latency (ms)',
                  fontsize=FONT_SIZE_LABEL)
    # ax.set_title('Block Size vs Goal-to-Finish Latency', fontsize=FONT_SIZE_TITLE)
    ax.set_xticks(x)
    ax.set_xticklabels(all_block_sizes, fontsize=FONT_SIZE_TICK_LABELS)
    ax.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
    ax.yaxis.get_offset_text().set_fontsize(FONT_SIZE_OFFSET)
    # ax.legend(fontsize=LEGEND_SIZE)  # Legend removed
    ax.grid(True, axis='y')

    plt.tight_layout(pad=0)
    plt.savefig(PLOTS_DIR / 'goal_to_finish_latency.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()

    # Plot 6: Cancel Latency (Goal to Cancel)
    print("  - Goal to cancel latency")
    fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT))

    # Get all unique block sizes present in the data
    all_block_sizes = sorted(df['block_size'].unique())
    x = np.arange(len(all_block_sizes))
    width = 0.2

    for i, mode in enumerate(['reactive', 'proactive']):
        for j, threading in enumerate(['single', 'multi']):
            data = df[(df['mode'] == mode) & (df['threading'] == threading)]
            data = data.sort_values('block_size')

            if data.empty:
                continue

            # Create aligned arrays for x positions and y values
            y_values = []
            yerr_values = []
            x_positions = []
            for block_size in all_block_sizes:
                block_data = data[data['block_size'] == block_size]
                if not block_data.empty:
                    y_values.append(
                        block_data['avg_goal_to_cancel_latency'].iloc[0])
                    yerr_values.append(
                        block_data['std_goal_to_cancel_latency'].iloc[0])
                    x_positions.append(all_block_sizes.index(block_size))

            if y_values:
                offset = (i * 2 + j - 1.5) * width
                ax.bar(np.array(x_positions) + offset, y_values, width,
                       label=f'{mode}-{threading}', yerr=yerr_values, capsize=CAPSIZE)

    ax.set_xlabel('Block Size', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Average Goal-to-Cancel Latency (ms)',
                  fontsize=FONT_SIZE_LABEL)
    # ax.set_title('Block Size vs Goal-to-Cancel Latency', fontsize=FONT_SIZE_TITLE)
    ax.set_xticks(x)
    ax.set_xticklabels(all_block_sizes, fontsize=FONT_SIZE_TICK_LABELS)
    ax.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
    ax.yaxis.get_offset_text().set_fontsize(FONT_SIZE_OFFSET)
    # ax.legend(fontsize=LEGEND_SIZE)  # Legend removed
    ax.grid(True, axis='y')

    plt.tight_layout(pad=0)
    plt.savefig(PLOTS_DIR / 'goal_to_cancel_latency.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()

    # Plot 7: Cancel to Finish Latency
    print("  - Cancel to finish latency")
    fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT))

    # Get all unique block sizes present in the data
    all_block_sizes = sorted(df['block_size'].unique())
    x = np.arange(len(all_block_sizes))
    width = 0.2

    for i, mode in enumerate(['reactive', 'proactive']):
        for j, threading in enumerate(['single', 'multi']):
            data = df[(df['mode'] == mode) & (df['threading'] == threading)]
            data = data.sort_values('block_size')

            if data.empty:
                continue

            # Create aligned arrays for x positions and y values
            y_values = []
            yerr_values = []
            x_positions = []
            for block_size in all_block_sizes:
                block_data = data[data['block_size'] == block_size]
                if not block_data.empty:
                    y_values.append(
                        block_data['avg_cancel_to_finish_latency'].iloc[0])
                    yerr_values.append(
                        block_data['std_cancel_to_finish_latency'].iloc[0])
                    x_positions.append(all_block_sizes.index(block_size))

            if y_values:
                offset = (i * 2 + j - 1.5) * width
                ax.bar(np.array(x_positions) + offset, y_values, width,
                       label=f'{mode}-{threading}', yerr=yerr_values, capsize=CAPSIZE)

    ax.set_xlabel('Block Size', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Cancellation Latency (ms)',
                  fontsize=FONT_SIZE_LABEL)
    # ax.set_title('Block Size vs Cancellation Latency', fontsize=FONT_SIZE_TITLE)
    ax.set_xticks(x)
    ax.set_xticklabels(all_block_sizes, fontsize=FONT_SIZE_TICK_LABELS)
    ax.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
    ax.yaxis.get_offset_text().set_fontsize(FONT_SIZE_OFFSET)
    # ax.legend(fontsize=LEGEND_SIZE)  # Legend removed
    ax.grid(True, axis='y')

    plt.tight_layout(pad=0)
    plt.savefig(PLOTS_DIR / 'cancellation_latency.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()

    # Plot 8: Total Segments (block_size * num_blocks)
    print("  - Total Segments completed")
    fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT))

    # Get all unique block sizes present in the data
    all_block_sizes = sorted(df['block_size'].unique())
    x = np.arange(len(all_block_sizes))
    width = 0.2

    for i, mode in enumerate(['reactive', 'proactive']):
        for j, threading in enumerate(['single', 'multi']):
            data = df[(df['mode'] == mode) & (df['threading'] == threading)]
            data = data.sort_values('block_size')

            if data.empty:
                continue

            # Create aligned arrays for x positions and y values
            y_values = []
            x_positions = []
            for block_size in all_block_sizes:
                block_data = data[data['block_size'] == block_size]
                if not block_data.empty:
                    y_values.append(block_data['total_iterations'].iloc[0])
                    x_positions.append(all_block_sizes.index(block_size))

            if y_values:
                offset = (i * 2 + j - 1.5) * width
                ax.bar(np.array(x_positions) + offset, y_values, width,
                       label=f'{mode}-{threading}')

    ax.set_xlabel('Block Size', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Total Segments Completed', fontsize=FONT_SIZE_LABEL)
    # ax.set_title('Block Size vs Total Segments Completed', fontsize=FONT_SIZE_TITLE)
    ax.set_xticks(x)
    ax.set_xticklabels(all_block_sizes, fontsize=FONT_SIZE_TICK_LABELS)
    ax.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
    ax.yaxis.get_offset_text().set_fontsize(FONT_SIZE_OFFSET)
    # ax.legend(fontsize=LEGEND_SIZE)  # Legend removed
    ax.grid(True, axis='y')

    plt.tight_layout(pad=0)
    plt.savefig(PLOTS_DIR / 'total_iterations.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()

    # Plot 9: Total Cancellation Time (goal_to_cancel + server_cancel_response)
    print("  - Total cancellation time (start to cancel + cancellation delay)")

    # Calculate combined cancellation time
    df['total_cancellation_time'] = df['avg_goal_to_cancel_latency'] + \
        df['avg_server_cancel_response']

    fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT))

    # Get all unique block sizes present in the data
    all_block_sizes = sorted(df['block_size'].unique())
    x = np.arange(len(all_block_sizes))
    width = 0.2

    for i, mode in enumerate(['reactive', 'proactive']):
        for j, threading in enumerate(['single', 'multi']):
            data = df[(df['mode'] == mode) & (df['threading'] == threading)]
            data = data.sort_values('block_size')

            if data.empty:
                continue

            # Create aligned arrays for x positions and y values
            y_values = []
            x_positions = []
            for block_size in all_block_sizes:
                block_data = data[data['block_size'] == block_size]
                if not block_data.empty:
                    y_values.append(
                        block_data['total_cancellation_time'].iloc[0])
                    x_positions.append(all_block_sizes.index(block_size))

            if y_values:
                offset = (i * 2 + j - 1.5) * width
                ax.bar(np.array(x_positions) + offset, y_values, width,
                       label=f'{mode}-{threading}')

    ax.set_xlabel('Block Size', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Total Cancellation Time (ms)', fontsize=FONT_SIZE_LABEL)
    # ax.set_title(
    #     'Block Size vs Total Cancellation Time\n(Goal-to-Cancel + Server Cancel Response)', fontsize=FONT_SIZE_TITLE)
    ax.set_xticks(x)
    ax.set_xticklabels(all_block_sizes, fontsize=FONT_SIZE_TICK_LABELS)
    ax.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
    ax.yaxis.get_offset_text().set_fontsize(FONT_SIZE_OFFSET)
    # ax.legend(fontsize=LEGEND_SIZE)  # Legend removed
    ax.grid(True, axis='y')

    plt.tight_layout(pad=0)
    plt.savefig(PLOTS_DIR / 'total_cancellation_time.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()

    # Create a separate legend-only plot with 4 columns in 1 row
    print("  - Creating separate legend")

    # Create dummy patches for the legend
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor=f'C{i*2+j}', label=f'{mode}-{threading}')
        for i, mode in enumerate(['reactive', 'proactive'])
        for j, threading in enumerate(['single', 'multi'])
    ]

    # Create a minimal figure just for the legend
    fig_legend = plt.figure(figsize=(PLOT_WIDTH, 0.5))
    legend = fig_legend.legend(
        handles=legend_elements,
        loc='center',
        ncol=4,
        fontsize=LEGEND_SIZE,
        frameon=False
    )

    # Save with minimal margins
    plt.savefig(PLOTS_DIR / 'legend.pdf', dpi=PLOT_DPI,
                bbox_inches='tight', pad_inches=0)
    plt.close()

    print(f"  All plots saved to: {PLOTS_DIR}")


def process_single_trace(trace_dir):
    """Process a single trace directory (for parallel execution)"""
    config_name = trace_dir.name
    events = parse_trace_directory(trace_dir)

    if not events:
        print(f"  Warning: No events found in {config_name}")
        return None

    metrics = extract_metrics_from_events(events, config_name)
    print(f"    {config_name}: Blocks: {metrics['total_blocks']}, "
          f"Iterations: {metrics['total_iterations']}, "
          f"Avg time/block: {metrics['avg_time_per_block']:.2f}ms")

    return metrics


def main():
    print("========================================")
    print("Monte Carlo Experiment Evaluation")
    print("========================================")
    print()

    # Find all trace directories (only block_*_run* experiment traces)
    trace_dirs = sorted([
        d for d in TRACE_DIR.iterdir()
        if d.is_dir() and d.name.startswith('block_')
    ])

    print(f"Looking for traces in: {TRACE_DIR}\n")
    for trace_dir in trace_dirs:
        print(f"  Found trace directory: {trace_dir.name}")

    if not trace_dirs:
        print(f"Error: No trace directories found in {TRACE_DIR}")
        return 1

    print(f"Found {len(trace_dirs)} trace directories\n")

    # Parse all traces
    all_metrics = []
    for trace_dir in trace_dirs:
        config_name = trace_dir.name
        events = parse_trace_directory(trace_dir)

        if not events:
            print(f"  Warning: No events found in {config_name}")
            continue

        metrics = extract_metrics_from_events(events, config_name)
        all_metrics.append(metrics)

        print(f"    Blocks: {metrics['total_blocks']}, "
              f"Iterations: {metrics['total_iterations']}, "
              f"Avg time/block: {metrics['avg_time_per_block']:.2f}ms")

    if not all_metrics:
        print("\nError: No metrics could be extracted from traces")
        return 1

    # Save individual run metrics
    print(f"\nSaving individual run metrics...")
    individual_df = pd.DataFrame(all_metrics)
    individual_csv = RESULTS_DIR / 'individual_runs.csv'
    individual_df.to_csv(individual_csv, index=False)
    print(f"  Saved to: {individual_csv}")

    # Aggregate runs
    print(f"\nAggregating metrics across runs...")
    aggregated = aggregate_runs(all_metrics)

    # Save aggregated metrics
    aggregated_df = pd.DataFrame([
        {**parse_config_name(config), **metrics}
        for config, metrics in aggregated.items()
    ])
    aggregated_csv = RESULTS_DIR / 'aggregated_results.csv'
    aggregated_df.to_csv(aggregated_csv, index=False)
    print(f"  Saved to: {aggregated_csv}")

    # Save as JSON as well
    aggregated_json = RESULTS_DIR / 'aggregated_results.json'
    with open(aggregated_json, 'w') as f:
        json.dump(aggregated, f, indent=2)
    print(f"  Saved to: {aggregated_json}")

    # Generate plots
    generate_plots(aggregated)

    # Print summary
    print("\n========================================")
    print("Summary Statistics")
    print("========================================")
    print(f"\nTotal configurations tested: {len(aggregated)}")
    print(f"Total experiment runs: {len(all_metrics)}")

    print("\nBlock Size Performance:")
    for block_size in sorted(aggregated_df['block_size'].unique()):
        data = aggregated_df[aggregated_df['block_size'] == block_size]
        avg_throughput = (data['avg_iterations_per_block'] /
                          (data['avg_time_per_block'] / 1000.0)).mean()
        print(
            f"  Block {block_size:6d}: {avg_throughput:10.2f} iterations/sec (avg)")

    print("\n========================================")
    print("Evaluation Complete!")
    print("========================================")
    print(f"\nResults directory: {RESULTS_DIR}")
    print(f"Plots directory: {PLOTS_DIR}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
