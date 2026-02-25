#!/usr/bin/env python3
"""
Step 4: Throughput Analysis

Analyzes throughput traces (from Step 3) to evaluate computation times across
different configurations (sync/async, single/multi-threaded).

Metrics analyzed:
- Total processing time per image/goal
- Layer computation times
- Exit calculation times
- Complete computation time (layer + exit stacked)
- Throughput comparison (images/second)

Input:  traces/phase3_{sync|async}_{single|multi}_trial{1,2,3}/
Output: results/runtime_analysis/
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

# Configuration
SCRIPT_DIR = Path(__file__).resolve().parent
EXPERIMENT_DIR = SCRIPT_DIR
TRACE_DIR = EXPERIMENT_DIR / "traces"
RESULTS_DIR = EXPERIMENT_DIR / "results"
RUNTIME_DIR = RESULTS_DIR / "runtime_analysis"

# Create output directories
RESULTS_DIR.mkdir(exist_ok=True)
RUNTIME_DIR.mkdir(exist_ok=True)


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

        try:
            # Extract timestamp (in brackets)
            ts_start = line.find('[')
            ts_end = line.find(']')
            if ts_start == -1 or ts_end == -1:
                continue

            timestamp_str = line[ts_start+1:ts_end]
            # Convert HH:MM:SS.nanoseconds to nanoseconds
            parts = timestamp_str.split(':')
            try:
                if len(parts) == 3:
                    hh = int(parts[0])
                    mm = int(parts[1])
                    ss = float(parts[2])
                    seconds_total = hh * 3600 + mm * 60 + ss
                    timestamp = seconds_total * 1e9
                else:
                    timestamp = float(timestamp_str) * 1e9
            except ValueError:
                continue

            # Extract event name
            event_start = line.find('anytime:')
            if event_start == -1:
                continue

            event_name_part = line[event_start:]
            event_name_end = event_name_part.find(
                ':', 8)  # Find colon after 'anytime:'
            if event_name_end == -1:
                continue

            event_name = event_name_part[8:event_name_end]

            # Extract fields (in braces)
            fields_start = line.find('{', event_start)
            fields_end = line.rfind('}')
            if fields_start == -1 or fields_end == -1:
                fields = {}
            else:
                fields_str = line[fields_start+1:fields_end]
                fields = parse_fields(fields_str)

            events.append(TraceEvent(timestamp, event_name, fields))

        except Exception as e:
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
        elif char == ',' and not in_string:
            parts.append(''.join(current).strip())
            current = []
            continue
        current.append(char)

    for part in parts:
        if not part or '=' not in part:
            continue

        key, value = part.split('=', 1)
        key = key.strip()
        value = value.strip()

        if value.startswith('"') and value.endswith('"'):
            value = value[1:-1]
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
    Expected format: phase3_{sync/async}_{single/multi}_trial{n}
    """
    config = {
        'sync_mode': None,
        'threading': None,
        'trial': None,
        'description': trace_name
    }

    if 'sync' in trace_name and 'async' not in trace_name:
        config['sync_mode'] = 'sync'
    elif 'async' in trace_name:
        config['sync_mode'] = 'async'

    if 'single' in trace_name:
        config['threading'] = 'single'
    elif 'multi' in trace_name:
        config['threading'] = 'multi'

    # Extract trial number
    if 'trial' in trace_name:
        try:
            trial_str = trace_name.split('trial')[1]
            config['trial'] = int(''.join(filter(str.isdigit, trial_str)))
        except:
            pass

    return config


def analyze_runtime_trace(trace_dir):
    """
    Analyze runtime metrics from a Phase 3 trace
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
        'goal_end': None,
        'layer_computation_times': {},  # layer_num -> time (ms)
        'exit_calculation_times': {},  # layer_num -> time (ms)
        'layer_start_times': {},
        'exit_calc_start_times': {},
        'total_layers': 0,
        'last_layer_end_time': None,  # Track when the last layer finished
        'final_exit_cost': None,  # Time from last layer to result
    }

    goal_counter = 0

    for event in events:
        if event.event_name == 'anytime_base_activate':
            # New goal started
            if current_goal['goal_start']:
                goals.append(current_goal)

            goal_counter += 1
            current_goal = {
                'goal_id': goal_counter,
                'goal_start': event.timestamp,
                'goal_end': None,
                'layer_computation_times': {},
                'exit_calculation_times': {},
                'layer_start_times': {},
                'exit_calc_start_times': {},
                'total_layers': 0,
                'last_layer_end_time': None,
                'final_exit_cost': None,
            }

        elif event.event_name == 'yolo_layer_start':
            layer_num = event.fields.get('layer_num', 0)
            current_goal['layer_start_times'][layer_num] = event.timestamp

        elif event.event_name == 'yolo_layer_end':
            layer_num = event.fields.get('layer_num', 0)
            # yolo_layer_start uses layer_num-1, yolo_layer_end uses layer_num
            start_layer_num = layer_num - 1
            if start_layer_num in current_goal['layer_start_times']:
                start_time = current_goal['layer_start_times'][start_layer_num]
                computation_time = (
                    event.timestamp - start_time) / 1e6  # Convert to ms
                current_goal['layer_computation_times'][layer_num] = computation_time
            current_goal['total_layers'] = max(
                current_goal['total_layers'], layer_num)
            # Track the timestamp of the last layer end
            current_goal['last_layer_end_time'] = event.timestamp

        elif event.event_name == 'yolo_cuda_callback':
            # In async mode, CUDA callbacks indicate layer completion
            # processed_layers = N means layers 0 through N-1 are complete
            processed_layers = event.fields.get('processed_layers', 0)
            if processed_layers > 0:
                layer_num = processed_layers - 1  # The layer that just completed
                # Find the most recent yolo_layer_start for this layer
                if layer_num in current_goal['layer_start_times']:
                    start_time = current_goal['layer_start_times'][layer_num]
                    computation_time = (
                        event.timestamp - start_time) / 1e6  # Convert to ms
                    # Store with layer_num+1 to match sync mode numbering (layer_end uses layer_num)
                    current_goal['layer_computation_times'][processed_layers] = computation_time
                current_goal['total_layers'] = max(
                    current_goal['total_layers'], processed_layers)
                # Track the timestamp of the last layer end (for async mode)
                current_goal['last_layer_end_time'] = event.timestamp

        elif event.event_name == 'yolo_exit_calculation_start':
            layer_num = event.fields.get('layer_num', 0)
            current_goal['exit_calc_start_times'][layer_num] = event.timestamp

        elif event.event_name == 'yolo_exit_calculation_end':
            layer_num = event.fields.get('layer_num', 0)
            if layer_num in current_goal['exit_calc_start_times']:
                start_time = current_goal['exit_calc_start_times'][layer_num]
                calc_time = (event.timestamp - start_time) / \
                    1e6  # Convert to ms
                current_goal['exit_calculation_times'][layer_num] = calc_time

        elif event.event_name == 'yolo_result':
            # Goal completed (Phase 1 style with exit calculations)
            current_goal['goal_end'] = event.timestamp
            # Calculate final exit cost (time from last layer to result)
            if current_goal['last_layer_end_time']:
                current_goal['final_exit_cost'] = (
                    event.timestamp - current_goal['last_layer_end_time']) / 1e6  # Convert to ms

        elif event.event_name == 'anytime_compute_exit':
            # Goal completed (Phase 3 style - batch processing, no intermediate exits)
            current_goal['goal_end'] = event.timestamp
            # Calculate final exit cost (time from last layer to result)
            if current_goal['last_layer_end_time']:
                current_goal['final_exit_cost'] = (
                    event.timestamp - current_goal['last_layer_end_time']) / 1e6  # Convert to ms
            # Optionally extract total computation time from the event
            # computation_time_ns = event.fields.get('computation_time_ns', 0)
            # iterations_completed = event.fields.get('iterations_completed', 0)

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


def aggregate_metrics(all_metrics):
    """
    Aggregate metrics across all traces and group by configuration
    """
    print("\n  Aggregating metrics by configuration...")

    # Group by configuration (sync_mode + threading)
    config_groups = defaultdict(lambda: {
        'traces': [],
        'total_goal_times': [],  # Total time per goal (ms)
        'layer_computation_times': defaultdict(list),  # layer -> [times]
        'exit_calculation_times': defaultdict(list),  # layer -> [times]
        'total_times_per_layer': defaultdict(list),  # layer -> [comp + exit]
        'final_exit_costs': [],  # Final exit cost per goal (ms)
    })

    for metrics in all_metrics:
        if not metrics:
            continue

        config = metrics['config']
        key = f"{config['sync_mode']}_{config['threading']}"
        group = config_groups[key]

        group['traces'].append(metrics['trace_name'])

        for goal in metrics['goals']:
            # Calculate total goal time
            if goal['goal_start'] and goal['goal_end']:
                total_time = (goal['goal_end'] - goal['goal_start']) / 1e6
                group['total_goal_times'].append(total_time)

            # Aggregate layer times
            for layer_num, comp_time in goal['layer_computation_times'].items():
                group['layer_computation_times'][layer_num].append(comp_time)

            for layer_num, exit_time in goal['exit_calculation_times'].items():
                group['exit_calculation_times'][layer_num].append(exit_time)

            # Calculate total time per layer (comp + exit)
            for layer_num in goal['layer_computation_times'].keys():
                comp_time = goal['layer_computation_times'].get(layer_num, 0)
                exit_time = goal['exit_calculation_times'].get(layer_num, 0)
                total = comp_time + exit_time
                group['total_times_per_layer'][layer_num].append(total)

            # Aggregate final exit costs
            if goal['final_exit_cost'] is not None:
                group['final_exit_costs'].append(goal['final_exit_cost'])

    # Calculate statistics for each group
    summary = {}
    for key, group in config_groups.items():
        summary[key] = {
            'config': key,
            'traces': group['traces'],
            'num_goals': len(group['total_goal_times']),
            'avg_goal_time_ms': np.mean(group['total_goal_times']) if group['total_goal_times'] else 0,
            'std_goal_time_ms': np.std(group['total_goal_times']) if group['total_goal_times'] else 0,
            'min_goal_time_ms': np.min(group['total_goal_times']) if group['total_goal_times'] else 0,
            'max_goal_time_ms': np.max(group['total_goal_times']) if group['total_goal_times'] else 0,
            'throughput_imgs_per_sec': len(group['total_goal_times']) / (sum(group['total_goal_times']) / 1000) if group['total_goal_times'] else 0,
        }

        # Layer-wise statistics
        summary[key]['layer_comp_stats'] = {}
        for layer, times in group['layer_computation_times'].items():
            if times:
                summary[key]['layer_comp_stats'][layer] = {
                    'mean': np.mean(times),
                    'std': np.std(times),
                    'min': np.min(times),
                    'max': np.max(times),
                }

        summary[key]['layer_exit_stats'] = {}
        for layer, times in group['exit_calculation_times'].items():
            if times:
                summary[key]['layer_exit_stats'][layer] = {
                    'mean': np.mean(times),
                    'std': np.std(times),
                    'min': np.min(times),
                    'max': np.max(times),
                }

        summary[key]['layer_total_stats'] = {}
        for layer, times in group['total_times_per_layer'].items():
            if times:
                summary[key]['layer_total_stats'][layer] = {
                    'mean': np.mean(times),
                    'std': np.std(times),
                    'min': np.min(times),
                    'max': np.max(times),
                }

        # Final exit cost statistics
        if group['final_exit_costs']:
            summary[key]['final_exit_cost_stats'] = {
                'mean': np.mean(group['final_exit_costs']),
                'std': np.std(group['final_exit_costs']),
                'min': np.min(group['final_exit_costs']),
                'max': np.max(group['final_exit_costs']),
                'count': len(group['final_exit_costs']),
            }
        else:
            summary[key]['final_exit_cost_stats'] = None

    return summary


def plot_total_goal_time_comparison(summary):
    """
    Plot comparison of total goal processing times across configurations
    """
    print("\n  Creating total goal time comparison plot...")

    configs = sorted(summary.keys())
    if not configs:
        print("    No data available")
        return

    means = [summary[c]['avg_goal_time_ms'] for c in configs]
    stds = [summary[c]['std_goal_time_ms'] for c in configs]
    labels = [c.replace('_', '+').replace('sync', 'Sync').replace(
        'async', 'Async').replace('single', 'Single').replace('multi', 'Multi') for c in configs]

    fig, ax = plt.subplots(figsize=(10, 6))

    x = np.arange(len(configs))
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
    bar_colors = [colors[i % len(colors)] for i in range(len(configs))]
    bars = ax.bar(x, means, yerr=stds, capsize=5, alpha=0.7, color=bar_colors)

    ax.set_xlabel('Configuration', fontsize=12)
    ax.set_ylabel('Average Goal Processing Time (ms)', fontsize=12)
    ax.set_title('Total Goal Processing Time by Configuration', fontsize=14)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, ha='right')
    ax.grid(True, alpha=0.3, axis='y')

    # Add value labels on bars
    for i, (bar, mean) in enumerate(zip(bars, means)):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height,
                f'{mean:.1f}ms',
                ha='center', va='bottom', fontsize=10)

    plt.tight_layout()
    plt.savefig(RUNTIME_DIR / 'total_goal_time_comparison.png', dpi=300)
    plt.close()
    print(f"    Saved: {RUNTIME_DIR / 'total_goal_time_comparison.png'}")


def plot_throughput_comparison(summary):
    """
    Plot throughput comparison across configurations
    """
    print("\n  Creating throughput comparison plot...")

    configs = sorted(summary.keys())
    if not configs:
        print("    No data available")
        return

    throughputs = [summary[c]['throughput_imgs_per_sec'] for c in configs]
    labels = [c.replace('_', '+').replace('sync', 'Sync').replace(
        'async', 'Async').replace('single', 'Single').replace('multi', 'Multi') for c in configs]

    fig, ax = plt.subplots(figsize=(10, 6))

    x = np.arange(len(configs))
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
    bar_colors = [colors[i % len(colors)] for i in range(len(configs))]
    bars = ax.bar(x, throughputs, alpha=0.7, color=bar_colors)

    ax.set_xlabel('Configuration', fontsize=12)
    ax.set_ylabel('Throughput (images/sec)', fontsize=12)
    ax.set_title('Processing Throughput by Configuration', fontsize=14)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, ha='right')
    ax.grid(True, alpha=0.3, axis='y')

    # Add value labels on bars
    for i, (bar, throughput) in enumerate(zip(bars, throughputs)):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height,
                f'{throughput:.2f}',
                ha='center', va='bottom', fontsize=10)

    plt.tight_layout()
    plt.savefig(RUNTIME_DIR / 'throughput_comparison.png', dpi=300)
    plt.close()
    print(f"    Saved: {RUNTIME_DIR / 'throughput_comparison.png'}")


def plot_layer_computation_times_by_config(summary):
    """
    Plot layer computation times for each configuration
    """
    print("\n  Creating layer computation times plot...")

    fig, ax = plt.subplots(figsize=(14, 7))

    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
    markers = ['o', 's', '^', 'D', 'v', 'p']

    for idx, (config, data) in enumerate(sorted(summary.items())):
        if not data['layer_comp_stats']:
            continue

        layers = sorted(data['layer_comp_stats'].keys())
        means = [data['layer_comp_stats'][l]['mean'] for l in layers]
        stds = [data['layer_comp_stats'][l]['std'] for l in layers]

        label = config.replace('_', '+').replace('sync', 'Sync').replace(
            'async', 'Async').replace('single', 'Single').replace('multi', 'Multi')

        ax.errorbar(layers, means, yerr=stds, marker=markers[idx % len(markers)],
                    label=label, capsize=3, linewidth=2, markersize=6,
                    color=colors[idx % len(colors)], alpha=0.8)

    ax.set_xlabel('Layer Number', fontsize=12)
    ax.set_ylabel('Computation Time (ms)', fontsize=12)
    ax.set_title('Layer Computation Time by Configuration', fontsize=14)
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(RUNTIME_DIR / 'layer_computation_by_config.png', dpi=300)
    plt.close()
    print(f"    Saved: {RUNTIME_DIR / 'layer_computation_by_config.png'}")


def plot_exit_calculation_times_by_config(summary):
    """
    Plot exit calculation times for each configuration, including final exit cost
    """
    print("\n  Creating exit calculation times plot...")

    # Check if any configuration has exit calculation data or final exit cost
    has_layer_exit_data = any(data['layer_exit_stats']
                              for data in summary.values())
    has_final_exit_data = any(data.get('final_exit_cost_stats')
                              for data in summary.values())

    if not has_layer_exit_data and not has_final_exit_data:
        print("    No exit calculation data available")
        # Create a placeholder plot with informational message
        fig, ax = plt.subplots(figsize=(14, 7))
        ax.text(0.5, 0.5,
                'Exit Calculation Times\n\nNo exit calculation data available',
                ha='center', va='center', fontsize=14, transform=ax.transAxes,
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.axis('off')
        plt.tight_layout()
        plt.savefig(RUNTIME_DIR / 'exit_calculation_by_config.png', dpi=300)
        plt.close()
        print(f"    Saved: {RUNTIME_DIR / 'exit_calculation_by_config.png'}")
        return

    # Create figure with two subplots if we have both types of data
    if has_layer_exit_data and has_final_exit_data:
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 7))
    else:
        fig, ax1 = plt.subplots(figsize=(14, 7))
        ax2 = None

    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
    markers = ['o', 's', '^', 'D', 'v', 'p']

    # Plot 1: Layer-wise exit calculations (if available)
    if has_layer_exit_data:
        for idx, (config, data) in enumerate(sorted(summary.items())):
            if not data['layer_exit_stats']:
                continue

            layers = sorted(data['layer_exit_stats'].keys())
            means = [data['layer_exit_stats'][l]['mean'] for l in layers]
            stds = [data['layer_exit_stats'][l]['std'] for l in layers]

            label = config.replace('_', '+').replace('sync', 'Sync').replace(
                'async', 'Async').replace('single', 'Single').replace('multi', 'Multi')

            ax1.errorbar(layers, means, yerr=stds, marker=markers[idx % len(markers)],
                         label=label, capsize=3, linewidth=2, markersize=6,
                         color=colors[idx % len(colors)], alpha=0.8)

        ax1.set_xlabel('Layer Number', fontsize=12)
        ax1.set_ylabel('Exit Calculation Time (ms)', fontsize=12)
        ax1.set_title('Per-Layer Exit Calculation Time', fontsize=14)
        ax1.legend(loc='best', fontsize=10)
        ax1.grid(True, alpha=0.3)
    else:
        ax1.text(0.5, 0.5, 'No per-layer exit data\n(Phase 3 batch mode)',
                 ha='center', va='center', transform=ax1.transAxes, fontsize=12)
        ax1.set_title('Per-Layer Exit Calculation Time', fontsize=14)

    # Plot 2: Final exit cost comparison (if available)
    if has_final_exit_data and ax2 is not None:
        configs_with_final_exit = []
        means_final = []
        stds_final = []

        for config in sorted(summary.keys()):
            data = summary[config]
            if data.get('final_exit_cost_stats'):
                configs_with_final_exit.append(config)
                means_final.append(data['final_exit_cost_stats']['mean'])
                stds_final.append(data['final_exit_cost_stats']['std'])

        if configs_with_final_exit:
            labels = [c.replace('_', '+').replace('sync', 'Sync').replace(
                'async', 'Async').replace('single', 'Single').replace('multi', 'Multi')
                for c in configs_with_final_exit]

            x = np.arange(len(configs_with_final_exit))
            bar_colors = [colors[i % len(colors)]
                          for i in range(len(configs_with_final_exit))]
            bars = ax2.bar(x, means_final, yerr=stds_final,
                           capsize=5, alpha=0.7, color=bar_colors)

            ax2.set_xlabel('Configuration', fontsize=12)
            ax2.set_ylabel('Final Exit Cost (ms)', fontsize=12)
            ax2.set_title('Final Exit Cost (Last Layer → Result)', fontsize=14)
            ax2.set_xticks(x)
            ax2.set_xticklabels(labels, rotation=45, ha='right')
            ax2.grid(True, alpha=0.3, axis='y')

            # Add value labels on bars
            for i, (bar, mean) in enumerate(zip(bars, means_final)):
                height = bar.get_height()
                ax2.text(bar.get_x() + bar.get_width()/2., height,
                         f'{mean:.2f}ms',
                         ha='center', va='bottom', fontsize=10)
    elif has_final_exit_data and ax2 is None:
        # Single plot showing just final exit cost
        configs_with_final_exit = []
        means_final = []
        stds_final = []

        for config in sorted(summary.keys()):
            data = summary[config]
            if data.get('final_exit_cost_stats'):
                configs_with_final_exit.append(config)
                means_final.append(data['final_exit_cost_stats']['mean'])
                stds_final.append(data['final_exit_cost_stats']['std'])

        if configs_with_final_exit:
            labels = [c.replace('_', '+').replace('sync', 'Sync').replace(
                'async', 'Async').replace('single', 'Single').replace('multi', 'Multi')
                for c in configs_with_final_exit]

            x = np.arange(len(configs_with_final_exit))
            bar_colors = [colors[i % len(colors)]
                          for i in range(len(configs_with_final_exit))]
            bars = ax1.bar(x, means_final, yerr=stds_final,
                           capsize=5, alpha=0.7, color=bar_colors)

            ax1.set_xlabel('Configuration', fontsize=12)
            ax1.set_ylabel('Final Exit Cost (ms)', fontsize=12)
            ax1.set_title('Final Exit Cost (Last Layer → Result)', fontsize=14)
            ax1.set_xticks(x)
            ax1.set_xticklabels(labels, rotation=45, ha='right')
            ax1.grid(True, alpha=0.3, axis='y')

            # Add value labels on bars
            for i, (bar, mean) in enumerate(zip(bars, means_final)):
                height = bar.get_height()
                ax1.text(bar.get_x() + bar.get_width()/2., height,
                         f'{mean:.2f}ms',
                         ha='center', va='bottom', fontsize=10)

    plt.tight_layout()
    plt.savefig(RUNTIME_DIR / 'exit_calculation_by_config.png', dpi=300)
    plt.close()
    print(f"    Saved: {RUNTIME_DIR / 'exit_calculation_by_config.png'}")


def plot_stacked_layer_times_by_config(summary):
    """
    Create stacked bar plots showing layer computation + exit calculation times
    One subplot per configuration
    """
    print("\n  Creating stacked layer times plot...")

    configs = sorted(summary.keys())
    if not configs:
        print("    No data available")
        return

    # Create subplots - dynamically size grid based on number of configs
    n_configs = len(configs)
    n_cols = 2
    n_rows = (n_configs + n_cols - 1) // n_cols  # Ceiling division
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(16, 6 * n_rows))

    # Handle case where we have only one row
    if n_rows == 1:
        axes = axes.reshape(1, -1)
    axes = axes.flatten()

    for idx, config in enumerate(configs):
        if idx >= len(axes):
            break
        ax = axes[idx]
        data = summary[config]

        if not data['layer_comp_stats']:
            ax.text(0.5, 0.5, 'No data', ha='center', va='center',
                    transform=ax.transAxes)
            continue

        # Get layers with computation data
        layers = sorted(data['layer_comp_stats'].keys())

        if not layers:
            ax.text(0.5, 0.5, 'No data', ha='center', va='center',
                    transform=ax.transAxes)
            continue

        comp_means = [data['layer_comp_stats'][l]['mean'] for l in layers]

        # Check if we have exit calculation data
        has_exit_data = bool(data['layer_exit_stats'])

        if has_exit_data:
            # Get exit times for layers that have both computation and exit data
            exit_means = []
            filtered_layers = []
            filtered_comp_means = []
            for i, layer in enumerate(layers):
                if layer in data['layer_exit_stats']:
                    filtered_layers.append(layer)
                    filtered_comp_means.append(comp_means[i])
                    exit_means.append(data['layer_exit_stats'][layer]['mean'])

            if filtered_layers:
                # Create stacked bar chart with both computation and exit
                ax.bar(filtered_layers, filtered_comp_means, label='Layer Computation',
                       alpha=0.8, color='#1f77b4')
                ax.bar(filtered_layers, exit_means, bottom=filtered_comp_means,
                       label='Exit Calculation', alpha=0.8, color='#ff7f0e')
            else:
                # Just show computation times
                ax.bar(layers, comp_means, label='Layer Computation',
                       alpha=0.8, color='#1f77b4')
        else:
            # No exit data - just show computation times
            ax.bar(layers, comp_means, label='Layer Computation',
                   alpha=0.8, color='#1f77b4')

        label = config.replace('_', '+').replace('sync', 'Sync').replace(
            'async', 'Async').replace('single', 'Single').replace('multi', 'Multi')

        ax.set_xlabel('Layer Number', fontsize=11)
        ax.set_ylabel('Time (ms)', fontsize=11)
        ax.set_title(f'{label}', fontsize=12, fontweight='bold')
        ax.legend(loc='upper right', fontsize=9)
        ax.grid(True, alpha=0.3, axis='y')

    # Hide any unused subplots
    for idx in range(len(configs), len(axes)):
        axes[idx].set_visible(False)

    plt.suptitle('Layer Processing Time Breakdown (Stacked)',
                 fontsize=14, fontweight='bold', y=0.995)
    plt.tight_layout()
    plt.savefig(RUNTIME_DIR / 'stacked_layer_times_by_config.png', dpi=300)
    plt.close()
    print(f"    Saved: {RUNTIME_DIR / 'stacked_layer_times_by_config.png'}")


def plot_combined_stacked_comparison(summary):
    """
    Create a single comparison plot showing stacked times for all configurations side-by-side
    """
    print("\n  Creating combined stacked comparison plot...")

    configs = sorted(summary.keys())
    if not configs:
        print("    No data available")
        return

    fig, ax = plt.subplots(figsize=(16, 8))

    # Get all unique layers across all configs (use computation layers as primary source)
    all_layers = set()
    for config in configs:
        data = summary[config]
        all_layers.update(data['layer_comp_stats'].keys())

    layers = sorted(all_layers)
    if not layers:
        print("    No common layer data")
        return

    x = np.arange(len(layers))
    width = 0.2  # Width of each bar
    colors_comp = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
    colors_exit = ['#aec7e8', '#ffbb78', '#98df8a', '#ff9896']

    for idx, config in enumerate(configs):
        data = summary[config]
        label = config.replace('_', '+').replace('sync', 'Sync').replace(
            'async', 'Async').replace('single', 'Single').replace('multi', 'Multi')

        comp_means = []
        exit_means = []

        for layer in layers:
            comp_mean = data['layer_comp_stats'].get(
                layer, {}).get('mean', 0)
            exit_mean = data['layer_exit_stats'].get(layer, {}).get('mean', 0)
            comp_means.append(comp_mean)
            exit_means.append(exit_mean)

        offset = (idx - len(configs)/2 + 0.5) * width

        # Plot stacked bars
        ax.bar(x + offset, comp_means, width,
               label=f'{label} (Comp)',
               alpha=0.8, color=colors_comp[idx % len(colors_comp)])

        # Only plot exit bars if there's any exit data
        if any(e > 0 for e in exit_means):
            ax.bar(x + offset, exit_means, width, bottom=comp_means,
                   label=f'{label} (Exit)',
                   alpha=0.6, color=colors_exit[idx % len(colors_exit)])

    ax.set_xlabel('Layer Number', fontsize=12)
    ax.set_ylabel('Time (ms)', fontsize=12)
    ax.set_title(
        'Layer Processing Time Comparison (All Configurations)', fontsize=14)
    ax.set_xticks(x)
    ax.set_xticklabels(layers)
    ax.legend(loc='upper right', fontsize=9, ncol=2)
    ax.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    plt.savefig(RUNTIME_DIR / 'combined_stacked_comparison.png', dpi=300)
    plt.close()
    print(f"    Saved: {RUNTIME_DIR / 'combined_stacked_comparison.png'}")


def plot_cumulative_runtime_by_config(summary):
    """
    Plot cumulative runtime showing total processing time up to each layer
    for each configuration. This shows the total cost if we cancel at each specific layer.
    """
    print("\n  Creating cumulative runtime plot...")

    configs = sorted(summary.keys())
    if not configs:
        print("    No data available")
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))

    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
    markers = ['o', 's', '^', 'D', 'v', 'p']
    linestyles = ['-', '--', '-.', ':', '-', '--']

    # Plot 1: Cumulative total time (computation + exit) for each config
    for idx, config in enumerate(configs):
        data = summary[config]

        if not data['layer_total_stats']:
            continue

        layers = sorted(data['layer_total_stats'].keys())
        total_means = [data['layer_total_stats'][l]['mean'] for l in layers]

        # Calculate cumulative sum
        cumulative_total = np.cumsum(total_means)

        label = config.replace('_', '+').replace('sync', 'Sync').replace(
            'async', 'Async').replace('single', 'Single').replace('multi', 'Multi')

        ax1.plot(layers, cumulative_total, marker=markers[idx % len(markers)],
                 linewidth=2, markersize=6, label=label,
                 color=colors[idx % len(
                     colors)], linestyle=linestyles[idx % len(linestyles)],
                 alpha=0.8)

    ax1.set_xlabel('Layer Number', fontsize=12)
    ax1.set_ylabel('Cumulative Time (ms)', fontsize=12)
    ax1.set_title(
        'Total Cumulative Runtime Up To Each Layer (By Configuration)', fontsize=13)
    ax1.legend(loc='best', fontsize=10)
    ax1.grid(True, alpha=0.3)

    # Plot 2: Stacked area showing cumulative contribution for one config (or best config)
    # Choose the config with best throughput or first one
    if configs:
        # Use first config for detailed breakdown
        config = configs[0]
        data = summary[config]

        if data['layer_comp_stats']:
            comp_layers = sorted(data['layer_comp_stats'].keys())
            if comp_layers:
                comp_means = [data['layer_comp_stats'][l]['mean'] for l in comp_layers]
                exit_means = [data['layer_exit_stats'].get(l, {}).get('mean', 0) for l in comp_layers]

                cumulative_comp = np.cumsum(comp_means)
                cumulative_exit = np.cumsum(exit_means)
                cumulative_total = cumulative_comp + cumulative_exit

                label = config.replace('_', '+').replace('sync', 'Sync').replace(
                    'async', 'Async').replace('single', 'Single').replace('multi', 'Multi')

                ax2.fill_between(comp_layers, 0, cumulative_comp,
                                 alpha=0.7, label='Layer Computation', color='#1f77b4')
                if any(e > 0 for e in exit_means):
                    ax2.fill_between(comp_layers, cumulative_comp, cumulative_total,
                                     alpha=0.7, label='Exit Calculation', color='#ff7f0e')
                ax2.plot(comp_layers, cumulative_total, marker='o',
                         linewidth=2, markersize=6, color='red', label='Total Runtime')

                ax2.set_xlabel('Layer Number', fontsize=12)
                ax2.set_ylabel('Cumulative Time (ms)', fontsize=12)
                ax2.set_title(
                    f'Cumulative Runtime Breakdown: {label}', fontsize=13)
                ax2.legend(loc='best', fontsize=10)
                ax2.grid(True, alpha=0.3)
            else:
                ax2.text(0.5, 0.5, 'Insufficient data for breakdown',
                         ha='center', va='center', transform=ax2.transAxes)
        else:
            ax2.text(0.5, 0.5, 'No computation data for breakdown',
                     ha='center', va='center', transform=ax2.transAxes)

    plt.tight_layout()
    plt.savefig(RUNTIME_DIR / 'cumulative_runtime.png', dpi=300)
    plt.close()
    print(f"    Saved: {RUNTIME_DIR / 'cumulative_runtime.png'}")


def export_runtime_results(summary):
    """
    Export runtime analysis results to JSON and text
    """
    print("\n  Exporting results...")

    # Helper to convert numpy types
    def convert_numpy(obj):
        if isinstance(obj, dict):
            return {k: convert_numpy(v) for k, v in obj.items()}
        elif isinstance(obj, (list, tuple)):
            return [convert_numpy(item) for item in obj]
        elif isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        return obj

    # Save JSON
    json_path = RUNTIME_DIR / 'runtime_analysis.json'
    with open(json_path, 'w') as f:
        json.dump(convert_numpy(summary), f, indent=2)
    print(f"    Saved: {json_path}")

    # Create human-readable summary
    summary_path = RUNTIME_DIR / 'runtime_summary.txt'
    with open(summary_path, 'w') as f:
        f.write("="*80 + "\n")
        f.write("YOLO PHASE 3 RUNTIME ANALYSIS SUMMARY\n")
        f.write("="*80 + "\n\n")

        f.write(
            f"Analysis Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")

        for config in sorted(summary.keys()):
            data = summary[config]
            label = config.replace('_', ' ').upper()

            f.write(f"\n{'='*80}\n")
            f.write(f"Configuration: {label}\n")
            f.write(f"{'='*80}\n")
            f.write(f"Traces analyzed: {len(data['traces'])}\n")
            for trace in data['traces']:
                f.write(f"  - {trace}\n")
            f.write(f"\nGoals processed: {data['num_goals']}\n\n")

            f.write("GOAL PROCESSING TIME:\n")
            f.write("-" * 80 + "\n")
            f.write(f"  Average: {data['avg_goal_time_ms']:.2f} ms\n")
            f.write(f"  Std Dev: {data['std_goal_time_ms']:.2f} ms\n")
            f.write(f"  Min:     {data['min_goal_time_ms']:.2f} ms\n")
            f.write(f"  Max:     {data['max_goal_time_ms']:.2f} ms\n\n")

            f.write(f"THROUGHPUT:\n")
            f.write("-" * 80 + "\n")
            f.write(
                f"  {data['throughput_imgs_per_sec']:.2f} images/second\n\n")

            f.write("LAYER COMPUTATION TIMES (mean ± std):\n")
            f.write("-" * 80 + "\n")
            for layer in sorted(data['layer_comp_stats'].keys()):
                stats = data['layer_comp_stats'][layer]
                f.write(
                    f"  Layer {layer:2d}: {stats['mean']:.3f} ± {stats['std']:.3f} ms\n")

            f.write("\nEXIT CALCULATION TIMES (mean ± std):\n")
            f.write("-" * 80 + "\n")
            for layer in sorted(data['layer_exit_stats'].keys()):
                stats = data['layer_exit_stats'][layer]
                f.write(
                    f"  Layer {layer:2d}: {stats['mean']:.3f} ± {stats['std']:.3f} ms\n")

            f.write("\nTOTAL TIME PER LAYER (Computation + Exit):\n")
            f.write("-" * 80 + "\n")
            for layer in sorted(data['layer_total_stats'].keys()):
                stats = data['layer_total_stats'][layer]
                comp = data['layer_comp_stats'].get(layer, {}).get('mean', 0)
                exit_time = data['layer_exit_stats'].get(
                    layer, {}).get('mean', 0)
                f.write(f"  Layer {layer:2d}: {stats['mean']:.3f} ms ")
                f.write(f"(comp: {comp:.3f} ms, exit: {exit_time:.3f} ms)\n")

            # Add final exit cost information
            if data.get('final_exit_cost_stats'):
                f.write("\nFINAL EXIT COST (Last Layer → Result):\n")
                f.write("-" * 80 + "\n")
                fec_stats = data['final_exit_cost_stats']
                f.write(f"  Average: {fec_stats['mean']:.3f} ms\n")
                f.write(f"  Std Dev: {fec_stats['std']:.3f} ms\n")
                f.write(f"  Min:     {fec_stats['min']:.3f} ms\n")
                f.write(f"  Max:     {fec_stats['max']:.3f} ms\n")
                f.write(f"  Count:   {fec_stats['count']} goals\n")

            f.write("\n")

        f.write("\n" + "="*80 + "\n")
        f.write("COMPARISON SUMMARY\n")
        f.write("="*80 + "\n\n")

        f.write("Average Goal Processing Time:\n")
        for config in sorted(summary.keys()):
            label = config.replace('_', '+').replace('sync', 'Sync').replace(
                'async', 'Async').replace('single', 'Single').replace('multi', 'Multi')
            f.write(
                f"  {label:20s}: {summary[config]['avg_goal_time_ms']:.2f} ms\n")

        f.write("\nThroughput:\n")
        for config in sorted(summary.keys()):
            label = config.replace('_', '+').replace('sync', 'Sync').replace(
                'async', 'Async').replace('single', 'Single').replace('multi', 'Multi')
            f.write(
                f"  {label:20s}: {summary[config]['throughput_imgs_per_sec']:.2f} imgs/sec\n")

        f.write("\nFinal Exit Cost (Last Layer → Result):\n")
        for config in sorted(summary.keys()):
            data = summary[config]
            label = config.replace('_', '+').replace('sync', 'Sync').replace(
                'async', 'Async').replace('single', 'Single').replace('multi', 'Multi')
            if data.get('final_exit_cost_stats'):
                f.write(
                    f"  {label:20s}: {data['final_exit_cost_stats']['mean']:.3f} ms\n")
            else:
                f.write(f"  {label:20s}: N/A\n")

        f.write("\n" + "="*80 + "\n")

    print(f"    Saved: {summary_path}")

    # Print to console
    with open(summary_path, 'r') as f:
        print("\n" + f.read())


def main():
    """Main runtime analysis function"""
    print("="*80)
    print("STEP 4: THROUGHPUT ANALYSIS")
    print("="*80)
    print(f"Input:  {TRACE_DIR}")
    print(f"Output: {RUNTIME_DIR}")

    # Find Phase 3 traces
    phase3_traces = [d for d in TRACE_DIR.iterdir()
                     if d.is_dir() and 'phase3' in d.name]

    if not phase3_traces:
        print("\n❌ Error: No Phase 3 traces found!")
        print(f"Please run Step 3 first: ./3_measure_throughput.sh")
        return 1

    print(f"\nFound {len(phase3_traces)} Phase 3 trace directories")

    # Analyze all Phase 3 traces
    print("\nAnalyzing runtime metrics...")
    all_metrics = []

    for trace_dir in phase3_traces:
        print(f"\n  Analyzing: {trace_dir.name}")
        metrics = analyze_runtime_trace(trace_dir)
        if metrics:
            all_metrics.append(metrics)
            print(f"    Processed {metrics['total_goals']} goals")

    if not all_metrics:
        print("\nError: No metrics extracted from traces!")
        return 1

    print(f"\nTotal traces analyzed: {len(all_metrics)}")

    # Aggregate metrics by configuration
    summary = aggregate_metrics(all_metrics)

    # Generate plots
    print("\nGenerating plots...")
    plot_total_goal_time_comparison(summary)
    plot_throughput_comparison(summary)
    plot_layer_computation_times_by_config(summary)
    plot_exit_calculation_times_by_config(summary)
    plot_stacked_layer_times_by_config(summary)
    plot_combined_stacked_comparison(summary)
    plot_cumulative_runtime_by_config(summary)

    # Export results
    export_runtime_results(summary)

    print("\n" + "="*80)
    print("✅ STEP 4 COMPLETE: THROUGHPUT ANALYSIS")
    print("="*80)
    print(f"Results: {RUNTIME_DIR}")
    print(f"\n⚠️  REVIEW OUTPUTS before proceeding!")
    print(f"\nNext steps:")
    print(f"  5. Generate configs: python3 5_generate_configs.py")
    print(f"  6. Run experiments:  ./6_run_experiments.sh")

    return 0


if __name__ == '__main__':
    sys.exit(main())
