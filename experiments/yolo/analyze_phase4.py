#!/usr/bin/env python3
"""
YOLO Phase 4 Analysis Script

This script analyzes Phase 4 traces to evaluate cancellation performance across different
configurations (block size, reactive/proactive, sync/async, single/multi-threaded).

Key Metrics:
1. Cancellation delay: Time from cancel request to result received
2. Total runtime: Time from goal start to result received
3. Number of layers processed: How many layers were computed before cancellation

Configurations tested:
- Block sizes: 1, 8, 25
- Modes: reactive, proactive
- Sync modes: sync, async
- Threading: single, multi
"""

import subprocess
import re
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from collections import defaultdict
import json
from datetime import datetime

# Configuration
EXPERIMENT_DIR = Path("/home/vscode/workspace/experiments/yolo")
TRACE_DIR = EXPERIMENT_DIR / "traces"
RESULTS_DIR = EXPERIMENT_DIR / "results"
PHASE4_DIR = RESULTS_DIR / "phase4_analysis"

# Create output directories
RESULTS_DIR.mkdir(exist_ok=True)
PHASE4_DIR.mkdir(exist_ok=True)


class TraceEvent:
    """Represents a single trace event"""

    def __init__(self, timestamp, event_name, fields):
        self.timestamp = timestamp
        self.event_name = event_name
        self.fields = fields

    def __repr__(self):
        return f"TraceEvent({self.timestamp}, {self.event_name}, {self.fields})"


def parse_trace_directory(trace_dir):
    """
    Parse a single trace directory using babeltrace
    """
    print(f"  Parsing trace: {trace_dir.name}")

    # Use babeltrace2 (without --names none to preserve all fields)
    try:
        result = subprocess.run(
            ['babeltrace2', str(trace_dir)],
            capture_output=True,
            text=True,
            check=True
        )
    except (subprocess.CalledProcessError, FileNotFoundError):
        # Try babeltrace (version 1) as fallback
        try:
            result = subprocess.run(
                ['babeltrace', str(trace_dir)],
                capture_output=True,
                text=True,
                check=True
            )
        except FileNotFoundError:
            print(f"    ERROR: babeltrace not found. Please install lttng-tools.")
            return []

    # Pre-filter lines to only process anytime events
    anytime_lines = [line for line in result.stdout.split(
        '\n') if 'anytime:' in line]

    events = []
    for line in anytime_lines:
        if not line.strip():
            continue

        try:
            # Extract timestamp (in brackets) - format: [HH:MM:SS.nanoseconds]
            ts_start = line.find('[')
            ts_end = line.find(']')
            if ts_start == -1 or ts_end == -1:
                continue

            timestamp_str = line[ts_start+1:ts_end]
            # Convert HH:MM:SS.nanoseconds to nanoseconds (int/float)
            parts = timestamp_str.split(':')
            try:
                if len(parts) == 3:
                    hh = int(parts[0])
                    mm = int(parts[1])
                    ss = float(parts[2])
                    seconds_total = hh * 3600 + mm * 60 + ss
                    timestamp = seconds_total * 1e9  # nanoseconds
                else:
                    timestamp = float(timestamp_str) * 1e9
            except ValueError:
                continue

            # Extract event name - everything after "anytime:" until the next colon
            event_start = line.find('anytime:')
            if event_start == -1:
                continue

            event_name_part = line[event_start:]
            event_name_end = event_name_part.find(
                ':', 8)  # Find colon after 'anytime:'
            if event_name_end == -1:
                continue

            event_name = 'anytime:' + event_name_part[8:event_name_end]

            # Extract fields (everything after the event name)
            fields_start = line.find('{')
            fields_end = line.rfind('}')
            if fields_start == -1 or fields_end == -1:
                fields = {}
            else:
                fields_str = line[fields_start+1:fields_end].strip()
                fields = parse_fields(fields_str)

            events.append(TraceEvent(timestamp, event_name, fields))

        except Exception as e:
            # Skip lines that can't be parsed
            continue

    print(f"    Parsed {len(events)} events")
    return events


def parse_fields(fields_str):
    """Parse the fields string into a dictionary"""
    fields = {}

    parts = []
    current = []
    in_string = False

    for char in fields_str + ',':
        if char == '"':
            in_string = not in_string
            current.append(char)
        elif char == ',' and not in_string:
            parts.append(''.join(current).strip())
            current = []
        else:
            current.append(char)

    for part in parts:
        if '=' not in part:
            continue

        key, value = part.split('=', 1)
        key = key.strip()
        value = value.strip()

        # Remove quotes from strings
        if value.startswith('"') and value.endswith('"'):
            value = value[1:-1]
        # Try to convert to appropriate type
        else:
            try:
                if '.' in value:
                    value = float(value)
                else:
                    value = int(value)
            except ValueError:
                pass

        fields[key] = value

    return fields


def parse_config_from_trace_name(trace_name):
    """
    Extract configuration from trace directory name
    Expected format: phase4_bs{size}_{mode}_{sync}_{threading}_trial{n}
    """
    config = {
        'block_size': None,
        'mode': None,
        'sync_mode': None,
        'threading': None,
        'trial': None,
        'description': trace_name
    }

    # Extract block size
    bs_match = re.search(r'bs(\d+)', trace_name)
    if bs_match:
        config['block_size'] = int(bs_match.group(1))

    # Extract mode
    if 'reactive' in trace_name:
        config['mode'] = 'reactive'
    elif 'proactive' in trace_name:
        config['mode'] = 'proactive'

    # Extract sync mode
    if 'async' in trace_name:
        config['sync_mode'] = 'async'
    elif 'sync' in trace_name:
        config['sync_mode'] = 'sync'

    # Extract threading
    if 'multi' in trace_name:
        config['threading'] = 'multi'
    elif 'single' in trace_name:
        config['threading'] = 'single'

    # Extract trial number
    trial_match = re.search(r'trial(\d+)', trace_name)
    if trial_match:
        config['trial'] = int(trial_match.group(1))

    return config


def analyze_cancellation_trace(trace_dir):
    """
    Analyze cancellation metrics from a Phase 4 trace
    Returns detailed timing metrics per goal
    """
    events = parse_trace_directory(trace_dir)

    if not events:
        return None

    config = parse_config_from_trace_name(trace_dir.name)

    # Track per-goal metrics
    goals = []
    current_goal = {
        'goal_id': 0,
        'goal_start': None,
        'cancel_sent': None,
        'result_received': None,
        'layers_processed': 0,
        'layer_start_times': {},
        'layer_end_times': {},
        'exit_calc_times': {},
    }

    goal_counter = 0

    for event in events:
        event_type = event.event_name

        if event_type == 'anytime:anytime_base_activate':
            # Start of new goal
            if current_goal['goal_start'] is not None:
                # Save previous goal
                goals.append(current_goal.copy())

            goal_counter += 1
            current_goal = {
                'goal_id': goal_counter,
                'goal_start': event.timestamp,
                'cancel_sent': None,
                'result_received': None,
                'layers_processed': 0,
                'layer_start_times': {},
                'layer_end_times': {},
                'exit_calc_times': {},
            }

        elif event_type == 'anytime:anytime_client_cancel_sent':
            # Cancel request sent
            current_goal['cancel_sent'] = event.timestamp

        elif event_type == 'anytime:anytime_client_goal_finished':
            # Result received
            current_goal['result_received'] = event.timestamp

        elif event_type == 'anytime:yolo_layer_start':
            # Layer computation started
            layer = event.fields.get('layer_num', -1)
            current_goal['layer_start_times'][layer] = event.timestamp

        elif event_type == 'anytime:anytime_compute_exit':
            # Layer computation ended (contains computation_time_ns)
            # Need to track which layer this corresponds to
            # We'll use the most recent layer_start
            if current_goal['layer_start_times']:
                layer = max(current_goal['layer_start_times'].keys())
                current_goal['layer_end_times'][layer] = event.timestamp
                current_goal['layers_processed'] = max(
                    current_goal['layers_processed'], layer + 1)

        elif event_type == 'anytime:yolo_exit_calculation_end':
            # Exit calculation ended
            layer_num = event.fields.get('layer_num', -1)
            current_goal['exit_calc_times'][layer_num] = event.timestamp

    # Don't forget the last goal
    if current_goal['goal_start']:
        goals.append(current_goal)

    # Calculate metrics
    metrics = {
        'trace_name': trace_dir.name,
        'config': config,
        'total_goals': len(goals),
        'goals': goals,
    }

    return metrics


def calculate_goal_metrics(goals):
    """
    Calculate cancellation delay, total runtime, and layers processed for each goal
    Returns list of metric dictionaries
    """
    goal_metrics = []

    for goal in goals:
        metrics = {
            'goal_id': goal['goal_id'],
            'cancellation_delay': None,  # Time from cancel to result
            'total_runtime': None,        # Time from goal start to result
            'layers_processed': goal['layers_processed'],
        }

        # Calculate cancellation delay
        if goal['cancel_sent'] and goal['result_received']:
            # timestamps are in nanoseconds -> convert to milliseconds
            metrics['cancellation_delay'] = (
                goal['result_received'] - goal['cancel_sent']) / 1e6  # ms

        # Calculate total runtime
        if goal['goal_start'] and goal['result_received']:
            # timestamps are in nanoseconds -> convert to milliseconds
            metrics['total_runtime'] = (
                goal['result_received'] - goal['goal_start']) / 1e6  # ms

        goal_metrics.append(metrics)

    return goal_metrics


def aggregate_metrics(all_metrics):
    """
    Aggregate metrics across all traces and group by configuration
    """
    print("\n  Aggregating metrics by configuration...")

    # Group by configuration (block_size + mode + sync_mode + threading)
    config_groups = defaultdict(lambda: {
        'traces': [],
        'cancellation_delays': [],      # ms
        'total_runtimes': [],            # ms
        'layers_processed': [],          # count
        'config': None,
    })

    for metrics in all_metrics:
        if not metrics:
            continue

        config = metrics['config']
        key = f"bs{config['block_size']}_{config['mode']}_{config['sync_mode']}_{config['threading']}"

        config_groups[key]['traces'].append(metrics['trace_name'])
        if config_groups[key]['config'] is None:
            config_groups[key]['config'] = config

        # Extract goal metrics
        goal_metrics = calculate_goal_metrics(metrics['goals'])

        for gm in goal_metrics:
            if gm['cancellation_delay'] is not None:
                config_groups[key]['cancellation_delays'].append(
                    gm['cancellation_delay'])
            if gm['total_runtime'] is not None:
                config_groups[key]['total_runtimes'].append(
                    gm['total_runtime'])
            config_groups[key]['layers_processed'].append(
                gm['layers_processed'])

    # Calculate statistics for each group
    summary = {}
    for key, group in config_groups.items():
        if not group['cancellation_delays'] and not group['total_runtimes']:
            continue

        summary[key] = {
            'config': group['config'],
            'num_traces': len(group['traces']),
            'num_goals': len(group['total_runtimes']),

            # Cancellation delay statistics
            'avg_cancellation_delay_ms': np.mean(group['cancellation_delays']) if group['cancellation_delays'] else None,
            'std_cancellation_delay_ms': np.std(group['cancellation_delays']) if group['cancellation_delays'] else None,
            'min_cancellation_delay_ms': np.min(group['cancellation_delays']) if group['cancellation_delays'] else None,
            'max_cancellation_delay_ms': np.max(group['cancellation_delays']) if group['cancellation_delays'] else None,
            'median_cancellation_delay_ms': np.median(group['cancellation_delays']) if group['cancellation_delays'] else None,

            # Total runtime statistics
            'avg_total_runtime_ms': np.mean(group['total_runtimes']) if group['total_runtimes'] else None,
            'std_total_runtime_ms': np.std(group['total_runtimes']) if group['total_runtimes'] else None,
            'min_total_runtime_ms': np.min(group['total_runtimes']) if group['total_runtimes'] else None,
            'max_total_runtime_ms': np.max(group['total_runtimes']) if group['total_runtimes'] else None,
            'median_total_runtime_ms': np.median(group['total_runtimes']) if group['total_runtimes'] else None,

            # Layers processed statistics
            'avg_layers_processed': np.mean(group['layers_processed']),
            'std_layers_processed': np.std(group['layers_processed']),
            'min_layers_processed': np.min(group['layers_processed']),
            'max_layers_processed': np.max(group['layers_processed']),
            'median_layers_processed': np.median(group['layers_processed']),

            # Raw data
            'cancellation_delays': group['cancellation_delays'],
            'total_runtimes': group['total_runtimes'],
            'layers_processed': group['layers_processed'],
        }

    return summary


def format_config_label(config_key):
    """Format configuration key into readable label"""
    parts = config_key.split('_')
    block_size = parts[0]  # bs1, bs8, bs25
    mode = parts[1].capitalize()  # Reactive/Proactive
    sync = parts[2].capitalize()  # Sync/Async
    threading = parts[3].capitalize()  # Single/Multi

    return f"{block_size.upper()} {mode} {sync} {threading}"


def plot_cancellation_delay_comparison(summary):
    """
    Plot comparison of cancellation delays across configurations
    """
    print("\n  Creating cancellation delay comparison plot...")

    configs = sorted(summary.keys())
    if not configs:
        print("    No data to plot")
        return

    # Filter out configs with no cancellation delay data
    configs = [c for c in configs if summary[c]
               ['avg_cancellation_delay_ms'] is not None]
    if not configs:
        print("    No cancellation delay data to plot")
        return

    means = [summary[c]['avg_cancellation_delay_ms'] for c in configs]
    stds = [summary[c]['std_cancellation_delay_ms'] for c in configs]
    labels = [format_config_label(c) for c in configs]

    fig, ax = plt.subplots(figsize=(16, 8))

    x = np.arange(len(configs))
    bars = ax.bar(x, means, yerr=stds, capsize=5, alpha=0.7)

    # Color bars by block size
    colors = {'bs1': '#1f77b4', 'bs8': '#ff7f0e', 'bs25': '#2ca02c'}
    for i, (bar, config) in enumerate(zip(bars, configs)):
        bs = config.split('_')[0]
        bar.set_color(colors.get(bs, '#888888'))

    ax.set_xlabel('Configuration', fontsize=12)
    ax.set_ylabel('Cancellation Delay (ms)', fontsize=12)
    ax.set_title('Cancellation Delay by Configuration', fontsize=14)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, ha='right', fontsize=8)
    ax.grid(True, alpha=0.3, axis='y')

    # Add legend for block sizes
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor=colors['bs1'], label='Block Size 1'),
        Patch(facecolor=colors['bs8'], label='Block Size 8'),
        Patch(facecolor=colors['bs25'], label='Block Size 25')
    ]
    ax.legend(handles=legend_elements, loc='upper left')

    plt.tight_layout()
    plt.savefig(PHASE4_DIR / 'cancellation_delay_comparison.png', dpi=300)
    plt.close()
    print(f"    Saved: {PHASE4_DIR / 'cancellation_delay_comparison.png'}")


def plot_total_runtime_comparison(summary):
    """
    Plot comparison of total runtimes across configurations
    """
    print("\n  Creating total runtime comparison plot...")

    configs = sorted(summary.keys())
    if not configs:
        print("    No data to plot")
        return

    means = [summary[c]['avg_total_runtime_ms'] for c in configs]
    stds = [summary[c]['std_total_runtime_ms'] for c in configs]
    labels = [format_config_label(c) for c in configs]

    fig, ax = plt.subplots(figsize=(16, 8))

    x = np.arange(len(configs))
    bars = ax.bar(x, means, yerr=stds, capsize=5, alpha=0.7)

    # Color bars by block size
    colors = {'bs1': '#1f77b4', 'bs8': '#ff7f0e', 'bs25': '#2ca02c'}
    for i, (bar, config) in enumerate(zip(bars, configs)):
        bs = config.split('_')[0]
        bar.set_color(colors.get(bs, '#888888'))

    ax.set_xlabel('Configuration', fontsize=12)
    ax.set_ylabel('Total Runtime (ms)', fontsize=12)
    ax.set_title('Total Runtime by Configuration', fontsize=14)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, ha='right', fontsize=8)
    ax.grid(True, alpha=0.3, axis='y')

    # Add legend for block sizes
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor=colors['bs1'], label='Block Size 1'),
        Patch(facecolor=colors['bs8'], label='Block Size 8'),
        Patch(facecolor=colors['bs25'], label='Block Size 25')
    ]
    ax.legend(handles=legend_elements, loc='upper left')

    plt.tight_layout()
    plt.savefig(PHASE4_DIR / 'total_runtime_comparison.png', dpi=300)
    plt.close()
    print(f"    Saved: {PHASE4_DIR / 'total_runtime_comparison.png'}")


def plot_layers_processed_comparison(summary):
    """
    Plot comparison of layers processed across configurations
    """
    print("\n  Creating layers processed comparison plot...")

    configs = sorted(summary.keys())
    if not configs:
        print("    No data to plot")
        return

    means = [summary[c]['avg_layers_processed'] for c in configs]
    stds = [summary[c]['std_layers_processed'] for c in configs]
    labels = [format_config_label(c) for c in configs]

    fig, ax = plt.subplots(figsize=(16, 8))

    x = np.arange(len(configs))
    bars = ax.bar(x, means, yerr=stds, capsize=5, alpha=0.7)

    # Color bars by block size
    colors = {'bs1': '#1f77b4', 'bs8': '#ff7f0e', 'bs25': '#2ca02c'}
    for i, (bar, config) in enumerate(zip(bars, configs)):
        bs = config.split('_')[0]
        bar.set_color(colors.get(bs, '#888888'))

    ax.set_xlabel('Configuration', fontsize=12)
    ax.set_ylabel('Layers Processed', fontsize=12)
    ax.set_title('Number of Layers Processed by Configuration', fontsize=14)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, ha='right', fontsize=8)
    ax.grid(True, alpha=0.3, axis='y')
    ax.axhline(y=16, color='r', linestyle='--',
               linewidth=2, label='Cancellation Threshold (16 layers)')

    # Add legend for block sizes
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor=colors['bs1'], label='Block Size 1'),
        Patch(facecolor=colors['bs8'], label='Block Size 8'),
        Patch(facecolor=colors['bs25'], label='Block Size 25'),
    ]
    ax.legend(handles=legend_elements, loc='upper left')

    plt.tight_layout()
    plt.savefig(PHASE4_DIR / 'layers_processed_comparison.png', dpi=300)
    plt.close()
    print(f"    Saved: {PHASE4_DIR / 'layers_processed_comparison.png'}")


def plot_metrics_by_block_size(summary):
    """
    Plot metrics grouped by block size
    """
    print("\n  Creating metrics by block size plot...")

    # Group by block size
    block_size_data = defaultdict(lambda: {
        'cancellation_delays': [],
        'total_runtimes': [],
        'layers_processed': [],
        'labels': []
    })

    for config_key, data in summary.items():
        bs = data['config']['block_size']
        mode = data['config']['mode']
        sync = data['config']['sync_mode']
        threading = data['config']['threading']
        label = f"{mode[0].upper()}/{sync[0].upper()}/{threading[0].upper()}"

        if data['avg_cancellation_delay_ms'] is not None:
            block_size_data[bs]['cancellation_delays'].append(
                data['avg_cancellation_delay_ms'])
        else:
            block_size_data[bs]['cancellation_delays'].append(0)

        block_size_data[bs]['total_runtimes'].append(
            data['avg_total_runtime_ms'])
        block_size_data[bs]['layers_processed'].append(
            data['avg_layers_processed'])
        block_size_data[bs]['labels'].append(label)

    # Create subplots
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    block_sizes = sorted(block_size_data.keys())
    colors_bs = {1: '#1f77b4', 8: '#ff7f0e', 25: '#2ca02c'}

    for idx, (ax, metric, title, ylabel) in enumerate([
        (axes[0], 'cancellation_delays',
         'Cancellation Delay', 'Delay (ms)'),
        (axes[1], 'total_runtimes', 'Total Runtime', 'Runtime (ms)'),
        (axes[2], 'layers_processed', 'Layers Processed', 'Layers'),
    ]):
        x_offset = 0
        for bs in block_sizes:
            data = block_size_data[bs][metric]
            labels = block_size_data[bs]['labels']
            x = np.arange(len(data)) + x_offset
            ax.bar(x, data, width=0.25, label=f'BS {bs}',
                   color=colors_bs[bs], alpha=0.7)
            x_offset += len(data) + 0.5

        ax.set_ylabel(ylabel, fontsize=11)
        ax.set_title(title, fontsize=12, fontweight='bold')
        ax.legend()
        ax.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    plt.savefig(PHASE4_DIR / 'metrics_by_block_size.png', dpi=300)
    plt.close()
    print(f"    Saved: {PHASE4_DIR / 'metrics_by_block_size.png'}")


def plot_distribution_boxplots(summary):
    """
    Create boxplots showing distribution of metrics
    """
    print("\n  Creating distribution boxplots...")

    fig, axes = plt.subplots(3, 1, figsize=(16, 12))

    configs = sorted(summary.keys())
    labels = [format_config_label(c) for c in configs]

    # Cancellation delay distribution
    cancel_data = [summary[c]['cancellation_delays']
                   for c in configs if summary[c]['cancellation_delays']]
    cancel_labels = [format_config_label(
        c) for c in configs if summary[c]['cancellation_delays']]

    if cancel_data:
        axes[0].boxplot(cancel_data, labels=cancel_labels)
        axes[0].set_ylabel('Cancellation Delay (ms)', fontsize=11)
        axes[0].set_title('Cancellation Delay Distribution',
                          fontsize=12, fontweight='bold')
        axes[0].tick_params(axis='x', rotation=45, labelsize=7)
        axes[0].grid(True, alpha=0.3, axis='y')

    # Total runtime distribution
    runtime_data = [summary[c]['total_runtimes'] for c in configs]
    axes[1].boxplot(runtime_data, labels=labels)
    axes[1].set_ylabel('Total Runtime (ms)', fontsize=11)
    axes[1].set_title('Total Runtime Distribution',
                      fontsize=12, fontweight='bold')
    axes[1].tick_params(axis='x', rotation=45, labelsize=7)
    axes[1].grid(True, alpha=0.3, axis='y')

    # Layers processed distribution
    layers_data = [summary[c]['layers_processed'] for c in configs]
    axes[2].boxplot(layers_data, labels=labels)
    axes[2].set_ylabel('Layers Processed', fontsize=11)
    axes[2].set_title('Layers Processed Distribution',
                      fontsize=12, fontweight='bold')
    axes[2].tick_params(axis='x', rotation=45, labelsize=7)
    axes[2].grid(True, alpha=0.3, axis='y')
    axes[2].axhline(y=16, color='r', linestyle='--',
                    linewidth=2, label='Cancellation Threshold')
    axes[2].legend()

    plt.tight_layout()
    plt.savefig(PHASE4_DIR / 'distribution_boxplots.png', dpi=300)
    plt.close()
    print(f"    Saved: {PHASE4_DIR / 'distribution_boxplots.png'}")


def export_phase4_results(summary):
    """
    Export Phase 4 results to JSON and text
    """
    print("\n  Exporting results...")

    # Helper to convert numpy types to native Python types
    def convert_numpy(obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, dict):
            return {key: convert_numpy(value) for key, value in obj.items()}
        elif isinstance(obj, list):
            return [convert_numpy(item) for item in obj]
        else:
            return obj

    # Create summary (without full raw data to keep JSON manageable)
    export_summary = {
        'analysis_date': datetime.now().isoformat(),
        'configurations': {}
    }

    for config_key, stats in summary.items():
        export_summary['configurations'][config_key] = {
            'config': stats['config'],
            'num_traces': stats['num_traces'],
            'num_goals': stats['num_goals'],
            'cancellation_delay': {
                'mean_ms': stats['avg_cancellation_delay_ms'],
                'std_ms': stats['std_cancellation_delay_ms'],
                'min_ms': stats['min_cancellation_delay_ms'],
                'max_ms': stats['max_cancellation_delay_ms'],
                'median_ms': stats['median_cancellation_delay_ms'],
            } if stats['avg_cancellation_delay_ms'] is not None else None,
            'total_runtime': {
                'mean_ms': stats['avg_total_runtime_ms'],
                'std_ms': stats['std_total_runtime_ms'],
                'min_ms': stats['min_total_runtime_ms'],
                'max_ms': stats['max_total_runtime_ms'],
                'median_ms': stats['median_total_runtime_ms'],
            },
            'layers_processed': {
                'mean': stats['avg_layers_processed'],
                'std': stats['std_layers_processed'],
                'min': stats['min_layers_processed'],
                'max': stats['max_layers_processed'],
                'median': stats['median_layers_processed'],
            }
        }

    export_summary = convert_numpy(export_summary)

    # Save JSON
    json_path = PHASE4_DIR / 'phase4_analysis.json'
    with open(json_path, 'w') as f:
        json.dump(export_summary, f, indent=2)
    print(f"    Saved: {json_path}")

    # Create human-readable summary
    summary_path = PHASE4_DIR / 'phase4_summary.txt'
    with open(summary_path, 'w') as f:
        f.write("="*80 + "\n")
        f.write("PHASE 4 CANCELLATION ANALYSIS SUMMARY\n")
        f.write("="*80 + "\n\n")
        f.write(
            f"Analysis Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(
            f"Total Configurations: {len(export_summary['configurations'])}\n\n")

        f.write("Client Configuration:\n")
        f.write("  - Cancel after layers: 16\n")
        f.write("  - Score threshold: 0.8\n")
        f.write("  - Target class: 9 (traffic light)\n\n")

        f.write("-"*80 + "\n\n")

        # Sort by total runtime
        sorted_configs = sorted(
            summary.items(),
            key=lambda x: x[1]['avg_total_runtime_ms']
        )

        for config_key, stats in sorted_configs:
            config = stats['config']
            f.write(f"Configuration: {format_config_label(config_key)}\n")
            f.write(f"  Block Size: {config['block_size']}\n")
            f.write(f"  Mode: {config['mode'].capitalize()}\n")
            f.write(f"  Sync: {config['sync_mode'].capitalize()}\n")
            f.write(f"  Threading: {config['threading'].capitalize()}\n")
            f.write(f"  Traces: {stats['num_traces']}\n")
            f.write(f"  Goals: {stats['num_goals']}\n\n")

            if stats['avg_cancellation_delay_ms'] is not None:
                f.write(f"  Cancellation Delay:\n")
                f.write(
                    f"    Mean: {stats['avg_cancellation_delay_ms']:.2f} ms\n")
                f.write(
                    f"    Std:  {stats['std_cancellation_delay_ms']:.2f} ms\n")
                f.write(
                    f"    Min:  {stats['min_cancellation_delay_ms']:.2f} ms\n")
                f.write(
                    f"    Max:  {stats['max_cancellation_delay_ms']:.2f} ms\n")
                f.write(
                    f"    Median: {stats['median_cancellation_delay_ms']:.2f} ms\n\n")

            f.write(f"  Total Runtime:\n")
            f.write(f"    Mean: {stats['avg_total_runtime_ms']:.2f} ms\n")
            f.write(f"    Std:  {stats['std_total_runtime_ms']:.2f} ms\n")
            f.write(f"    Min:  {stats['min_total_runtime_ms']:.2f} ms\n")
            f.write(f"    Max:  {stats['max_total_runtime_ms']:.2f} ms\n")
            f.write(
                f"    Median: {stats['median_total_runtime_ms']:.2f} ms\n\n")

            f.write(f"  Layers Processed:\n")
            f.write(f"    Mean: {stats['avg_layers_processed']:.2f}\n")
            f.write(f"    Std:  {stats['std_layers_processed']:.2f}\n")
            f.write(f"    Min:  {int(stats['min_layers_processed'])}\n")
            f.write(f"    Max:  {int(stats['max_layers_processed'])}\n")
            f.write(
                f"    Median: {stats['median_layers_processed']:.1f}\n\n")

            f.write("-"*80 + "\n\n")

        # Find best configurations
        f.write("\n" + "="*80 + "\n")
        f.write("BEST CONFIGURATIONS\n")
        f.write("="*80 + "\n\n")

        best_cancel = min(
            [s for s in summary.values() if s['avg_cancellation_delay_ms'] is not None],
            key=lambda x: x['avg_cancellation_delay_ms'],
            default=None
        )
        if best_cancel:
            f.write("Fastest Cancellation Delay:\n")
            f.write(
                f"  {format_config_label([k for k, v in summary.items() if v == best_cancel][0])}\n")
            f.write(
                f"  Delay: {best_cancel['avg_cancellation_delay_ms']:.2f} ms\n\n")

        best_runtime = min(
            summary.values(),
            key=lambda x: x['avg_total_runtime_ms']
        )
        f.write("Fastest Total Runtime:\n")
        f.write(
            f"  {format_config_label([k for k, v in summary.items() if v == best_runtime][0])}\n")
        f.write(
            f"  Runtime: {best_runtime['avg_total_runtime_ms']:.2f} ms\n\n")

        best_layers = min(
            summary.values(),
            key=lambda x: x['avg_layers_processed']
        )
        f.write("Fewest Layers Processed (Most Efficient):\n")
        f.write(
            f"  {format_config_label([k for k, v in summary.items() if v == best_layers][0])}\n")
        f.write(
            f"  Layers: {best_layers['avg_layers_processed']:.2f}\n\n")

    print(f"    Saved: {summary_path}")

    # Print to console
    print("\n" + "="*80)
    print("PHASE 4 ANALYSIS COMPLETE")
    print("="*80)
    with open(summary_path, 'r') as f:
        print(f.read())


def main():
    """Main Phase 4 analysis function"""
    print("="*80)
    print("YOLO PHASE 4 CANCELLATION ANALYSIS")
    print("="*80)
    print(f"Trace directory: {TRACE_DIR}")
    print(f"Results directory: {PHASE4_DIR}")

    # Find Phase 4 traces (only proactive architecture)
    phase4_traces = [d for d in TRACE_DIR.iterdir()
                     if d.is_dir() and 'phase4_' in d.name and 'proactive' in d.name]

    if not phase4_traces:
        print("\nERROR: No Phase 4 trace directories found!")
        print(f"Expected traces in: {TRACE_DIR}")
        print("Run experiments first: ./run_phase4_experiments.sh")
        return

    print(
        f"\nFound {len(phase4_traces)} Phase 4 trace directories (proactive only)")

    # Extract metrics from all traces
    print("\nAnalyzing traces...")
    all_metrics = []

    for trace_dir in sorted(phase4_traces):
        print(f"\nProcessing: {trace_dir.name}")
        metrics = analyze_cancellation_trace(trace_dir)
        if metrics:
            all_metrics.append(metrics)
            print(f"  Goals analyzed: {metrics['total_goals']}")

    if not all_metrics:
        print("\nERROR: No metrics extracted from traces!")
        return

    print(f"\nSuccessfully analyzed {len(all_metrics)} traces")

    # Aggregate metrics
    summary = aggregate_metrics(all_metrics)

    print(f"\nAggregated into {len(summary)} configurations")

    # Generate plots
    print("\nGenerating plots...")
    plot_cancellation_delay_comparison(summary)
    plot_total_runtime_comparison(summary)
    plot_layers_processed_comparison(summary)
    plot_metrics_by_block_size(summary)
    plot_distribution_boxplots(summary)

    # Export results
    export_phase4_results(summary)

    print("\n" + "="*80)
    print("ANALYSIS COMPLETE!")
    print("="*80)
    print(f"\nResults saved to: {PHASE4_DIR}")
    print("\nGenerated files:")
    print("  - phase4_analysis.json (machine-readable)")
    print("  - phase4_summary.txt (human-readable)")
    print("  - cancellation_delay_comparison.png")
    print("  - total_runtime_comparison.png")
    print("  - layers_processed_comparison.png")
    print("  - metrics_by_block_size.png")
    print("  - distribution_boxplots.png")


if __name__ == '__main__':
    main()
