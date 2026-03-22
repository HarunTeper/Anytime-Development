#!/usr/bin/env python3
"""
Step 7: Cancellation Performance Analysis

Analyzes cancellation traces (from Step 6) to evaluate responsiveness across
different configurations.

Key Metrics:
1. Cancellation delay: Time from cancel request to result received
2. Total runtime: Time from goal start to result received
3. Layers processed: How many layers were computed before cancellation

Configurations analyzed:
- Block sizes: 1, 4, 5, 8, 15, 22
- Mode: proactive
- Sync modes: sync, async
- Threading: single, multi

Input:  traces/phase4_bs{1,4,5,8,15,22}_proactive_{sync|async}_{single|multi}_trial{1,2,3,4,5}/
Output: results/phase4_analysis/
"""

import re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path
from collections import defaultdict
import json
import yaml
from datetime import datetime

from trace_utils import TraceEvent, parse_fields, parse_trace_directory

# Configuration
SCRIPT_DIR = Path(__file__).resolve().parent
EXPERIMENT_DIR = SCRIPT_DIR
TRACE_DIR = EXPERIMENT_DIR / "traces"
CONFIG_DIR = EXPERIMENT_DIR / "configs"
RESULTS_DIR = EXPERIMENT_DIR / "results"
PHASE4_DIR = RESULTS_DIR / "phase4_analysis"

# Number of warmup images to skip per trial (GPU JIT overhead)
WARMUP_IMAGES = 5

# Plot configuration
PLOT_WIDTH = 12
PLOT_HEIGHT = 8
PLOT_HEIGHT_SMALL = 8
PLOT_DPI = 300
FONT_SIZE_TITLE = 30
FONT_SIZE_LABEL = 30
FONT_SIZE_LEGEND = 30
FONT_SIZE_TICK_LABELS = 30
LEGEND_SIZE = 30
MARKER_SIZE = 12
CAPSIZE = 5
LINE_WIDTH = 2

# Create output directories
RESULTS_DIR.mkdir(exist_ok=True)
PHASE4_DIR.mkdir(exist_ok=True)


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
        'layer_detections': {},  # layer_num -> num_detections
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
                'layer_detections': {},  # layer_num -> num_detections
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

        elif event_type == 'anytime:yolo_layer_end':
            # Layer computation completed (fires once per layer in both sync and async modes)
            layer = event.fields.get('layer_num', -1)
            current_goal['layer_end_times'][layer] = event.timestamp
            current_goal['layers_processed'] = max(
                current_goal['layers_processed'], layer)

        elif event_type == 'anytime:yolo_exit_calculation_end':
            # Exit calculation ended
            layer_num = event.fields.get('layer_num', -1)
            num_detections = event.fields.get('num_detections', 0)
            current_goal['exit_calc_times'][layer_num] = event.timestamp
            current_goal['layer_detections'][layer_num] = num_detections

    # Don't forget the last goal
    if current_goal['goal_start']:
        goals.append(current_goal)

    # Skip warmup images (GPU JIT overhead in first few images)
    if WARMUP_IMAGES > 0 and len(goals) > WARMUP_IMAGES:
        goals = goals[WARMUP_IMAGES:]

    # Calculate metrics
    metrics = {
        'trace_name': trace_dir.name,
        'config': config,
        'total_goals': len(goals),
        'goals': goals,
    }

    return metrics


def calculate_goal_metrics(goals, cancel_after_layers=None):
    """
    Calculate cancellation delay, total runtime, layers processed, and detection
    count for each goal. Also classifies each goal as score-cancelled vs
    hard-deadline based on whether processing stopped before cancel_after_layers.
    """
    goal_metrics = []

    for goal in goals:
        metrics = {
            'goal_id': goal['goal_id'],
            'cancellation_delay': None,  # Time from cancel to result
            'total_runtime': None,        # Time from goal start to result
            'layers_processed': goal['layers_processed'],
            'score_cancelled': False,     # True if cancelled by score threshold
            'detections_at_cancel': None, # Detection count at cancellation point
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

        # Classify: score-cancelled if cancel was sent AND processing stopped
        # before the hard deadline layer count
        if cancel_after_layers is not None and goal['cancel_sent'] is not None:
            metrics['score_cancelled'] = (
                goal['layers_processed'] < cancel_after_layers)

        # Detection count at the highest processed layer
        max_layer = goal['layers_processed']
        metrics['detections_at_cancel'] = goal.get(
            'layer_detections', {}).get(max_layer, None)

        goal_metrics.append(metrics)

    return goal_metrics


def aggregate_metrics(all_metrics, cancel_after_layers=None):
    """
    Aggregate metrics across all traces and group by configuration.
    Separates score-cancelled goals from hard-deadline goals.
    """
    print("\n  Aggregating metrics by configuration...")

    # Group by configuration (block_size + mode + sync_mode + threading)
    config_groups = defaultdict(lambda: {
        'traces': [],
        'cancellation_delays': [],      # ms
        'total_runtimes': [],            # ms
        'layers_processed': [],          # count
        'detections_at_cancel': [],      # detection count at cancellation point
        'config': None,
        # Separated populations
        'score_cancelled_delays': [],    # ms (score-cancelled only)
        'score_cancelled_runtimes': [],  # ms (score-cancelled only)
        'score_cancelled_layers': [],    # count (score-cancelled only)
        'deadline_runtimes': [],         # ms (hard-deadline only)
        'deadline_layers': [],           # count (hard-deadline only)
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
        goal_metrics = calculate_goal_metrics(
            metrics['goals'], cancel_after_layers=cancel_after_layers)

        for gm in goal_metrics:
            if gm['cancellation_delay'] is not None:
                config_groups[key]['cancellation_delays'].append(
                    gm['cancellation_delay'])
            if gm['total_runtime'] is not None:
                config_groups[key]['total_runtimes'].append(
                    gm['total_runtime'])
            config_groups[key]['layers_processed'].append(
                gm['layers_processed'])
            if gm['detections_at_cancel'] is not None:
                config_groups[key]['detections_at_cancel'].append(
                    gm['detections_at_cancel'])

            # Separate populations
            if gm['score_cancelled']:
                if gm['cancellation_delay'] is not None:
                    config_groups[key]['score_cancelled_delays'].append(
                        gm['cancellation_delay'])
                if gm['total_runtime'] is not None:
                    config_groups[key]['score_cancelled_runtimes'].append(
                        gm['total_runtime'])
                config_groups[key]['score_cancelled_layers'].append(
                    gm['layers_processed'])
            else:
                if gm['total_runtime'] is not None:
                    config_groups[key]['deadline_runtimes'].append(
                        gm['total_runtime'])
                config_groups[key]['deadline_layers'].append(
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

            # Detection quality at cancellation point
            'avg_detections_at_cancel': np.mean(group['detections_at_cancel']) if group['detections_at_cancel'] else None,
            'std_detections_at_cancel': np.std(group['detections_at_cancel']) if group['detections_at_cancel'] else None,

            # Population breakdown
            'num_score_cancelled': len(group['score_cancelled_delays']),
            'num_deadline': len(group['deadline_runtimes']),
            'avg_score_cancelled_delay_ms': np.mean(group['score_cancelled_delays']) if group['score_cancelled_delays'] else None,
            'std_score_cancelled_delay_ms': np.std(group['score_cancelled_delays']) if group['score_cancelled_delays'] else None,
            'avg_score_cancelled_runtime_ms': np.mean(group['score_cancelled_runtimes']) if group['score_cancelled_runtimes'] else None,
            'avg_score_cancelled_layers': np.mean(group['score_cancelled_layers']) if group['score_cancelled_layers'] else None,
            'avg_deadline_runtime_ms': np.mean(group['deadline_runtimes']) if group['deadline_runtimes'] else None,
            'avg_deadline_layers': np.mean(group['deadline_layers']) if group['deadline_layers'] else None,

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
    print("  - Cancellation delay comparison")

    # Group data by block size
    block_sizes = sorted(set(s['config']['block_size']
                         for s in summary.values()))

    # Get all configuration combinations (sync, threading) - proactive only
    config_combos = []
    for sync in ['sync', 'async']:
        for threading in ['single', 'multi']:
            config_combos.append(('proactive', sync, threading))

    fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT))

    x = np.arange(len(block_sizes))
    width = 0.2  # Width of each bar (4 configs per block size)

    # Filter out configs with no cancellation delay data
    filtered_summary = {k: v for k, v in summary.items()
                        if v['avg_cancellation_delay_ms'] is not None}

    if not filtered_summary:
        print("    No cancellation delay data to plot")
        return

    # Define explicit colors for the 4 configurations
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c',
              '#d62728']  # blue, orange, green, red

    for i, (mode, sync, threading) in enumerate(config_combos):
        means = []
        stds = []

        for bs in block_sizes:
            # Find matching config
            key = f"bs{bs}_{mode}_{sync}_{threading}"
            if key in filtered_summary:
                means.append(filtered_summary[key]
                             ['avg_cancellation_delay_ms'])
                stds.append(filtered_summary[key]['std_cancellation_delay_ms'])
            else:
                means.append(0)
                stds.append(0)

        offset = (i - 1.5) * width  # Center the 4 bars around each x position
        ax.bar(x + offset, means, width, yerr=stds, capsize=CAPSIZE/2,
               color=colors[i], linewidth=LINE_WIDTH/2, label=f'{sync}-{threading}')

    ax.set_xlabel('Block Size', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Cancellation Delay (ms)', fontsize=FONT_SIZE_LABEL)
    ax.set_xticks(x)
    ax.set_xticklabels(block_sizes, fontsize=FONT_SIZE_TICK_LABELS)
    ax.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
    ax.grid(True, axis='y', alpha=0.3)

    plt.tight_layout(pad=0)
    plt.savefig(PHASE4_DIR / 'cancellation_delay_comparison.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()


def plot_total_runtime_comparison(summary):
    """
    Plot comparison of total runtimes across configurations
    """
    print("  - Total runtime comparison")

    # Group data by block size
    block_sizes = sorted(set(s['config']['block_size']
                         for s in summary.values()))

    # Get all configuration combinations (sync, threading) - proactive only
    config_combos = []
    for sync in ['sync', 'async']:
        for threading in ['single', 'multi']:
            config_combos.append(('proactive', sync, threading))

    fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT))

    x = np.arange(len(block_sizes))
    width = 0.2  # Width of each bar (4 configs per block size)

    # Define explicit colors for the 4 configurations
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c',
              '#d62728']  # blue, orange, green, red

    for i, (mode, sync, threading) in enumerate(config_combos):
        means = []
        stds = []

        for bs in block_sizes:
            # Find matching config
            key = f"bs{bs}_{mode}_{sync}_{threading}"
            if key in summary:
                means.append(summary[key]['avg_total_runtime_ms'])
                stds.append(summary[key]['std_total_runtime_ms'])
            else:
                means.append(0)
                stds.append(0)

        offset = (i - 1.5) * width  # Center the 4 bars around each x position
        ax.bar(x + offset, means, width, yerr=stds, capsize=CAPSIZE/2,
               color=colors[i], linewidth=LINE_WIDTH/2, label=f'{sync}-{threading}')

    ax.set_xlabel('Block Size', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Total Runtime (ms)', fontsize=FONT_SIZE_LABEL)
    ax.set_xticks(x)
    ax.set_xticklabels(block_sizes, fontsize=FONT_SIZE_TICK_LABELS)
    ax.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
    ax.grid(True, axis='y', alpha=0.3)

    plt.tight_layout(pad=0)
    plt.savefig(PHASE4_DIR / 'total_runtime_comparison.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()


def plot_layers_processed_comparison(summary, cancel_after_layers=None):
    """
    Plot comparison of layers processed across configurations
    """
    print("  - Layers processed comparison")

    # Group data by block size
    block_sizes = sorted(set(s['config']['block_size']
                         for s in summary.values()))

    # Get all configuration combinations (sync, threading) - proactive only
    config_combos = []
    for sync in ['sync', 'async']:
        for threading in ['single', 'multi']:
            config_combos.append(('proactive', sync, threading))

    fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT))

    x = np.arange(len(block_sizes))
    width = 0.2  # Width of each bar (4 configs per block size)

    # Define explicit colors for the 4 configurations
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c',
              '#d62728']  # blue, orange, green, red

    for i, (mode, sync, threading) in enumerate(config_combos):
        means = []
        stds = []

        for bs in block_sizes:
            # Find matching config
            key = f"bs{bs}_{mode}_{sync}_{threading}"
            if key in summary:
                means.append(summary[key]['avg_layers_processed'])
                stds.append(summary[key]['std_layers_processed'])
            else:
                means.append(0)
                stds.append(0)

        offset = (i - 1.5) * width  # Center the 4 bars around each x position
        ax.bar(x + offset, means, width, yerr=stds, capsize=CAPSIZE/2,
               color=colors[i], linewidth=LINE_WIDTH/2, label=f'{sync}-{threading}')

    ax.set_xlabel('Block Size', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Layers Processed', fontsize=FONT_SIZE_LABEL)
    ax.set_xticks(x)
    ax.set_xticklabels(block_sizes, fontsize=FONT_SIZE_TICK_LABELS)
    ax.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
    ax.grid(True, axis='y', alpha=0.3)
    if cancel_after_layers is not None:
        ax.axhline(y=cancel_after_layers, color='r', linestyle='--',
                   linewidth=LINE_WIDTH, label=f'Cancellation Threshold ({cancel_after_layers} layers)')

    plt.tight_layout(pad=0)
    plt.savefig(PHASE4_DIR / 'layers_processed_comparison.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()


def plot_metrics_by_block_size(summary):
    """
    Plot metrics grouped by block size
    """
    print("  - Metrics by block size")

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
    fig, axes = plt.subplots(1, 3, figsize=(18, PLOT_HEIGHT))

    block_sizes = sorted(block_size_data.keys())
    cmap = plt.cm.tab10
    colors_bs = {bs: cmap(i / max(len(block_sizes) - 1, 1))
                 for i, bs in enumerate(block_sizes)}

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
                   color=colors_bs[bs], alpha=0.7, linewidth=LINE_WIDTH)
            x_offset += len(data) + 0.5

        ax.set_ylabel(ylabel, fontsize=FONT_SIZE_LABEL)
        # ax.set_title(title, fontsize=FONT_SIZE_TITLE)
        ax.tick_params(axis='both', labelsize=FONT_SIZE_TICK_LABELS)
        # ax.legend(fontsize=LEGEND_SIZE)
        ax.grid(True, alpha=0.3, axis='y')

    plt.tight_layout(pad=0)
    plt.savefig(PHASE4_DIR / 'metrics_by_block_size.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()


def plot_distribution_boxplots(summary, cancel_after_layers=None):
    """
    Create boxplots showing distribution of metrics
    """
    print("  - Distribution boxplots")

    fig, axes = plt.subplots(3, 1, figsize=(PLOT_WIDTH + 4, PLOT_HEIGHT * 1.5))

    # Sort configs by block size, then mode, sync, threading
    def sort_key(config):
        parts = config.split('_')
        # Extract number from 'bs1', 'bs8', etc.
        block_size = int(parts[0][2:])
        return (block_size, parts[1], parts[2], parts[3])

    configs = sorted(summary.keys(), key=sort_key)
    labels = [format_config_label(c) for c in configs]

    # Cancellation delay distribution (keep sort order)
    cancel_configs = [c for c in configs if summary[c]['cancellation_delays']]
    cancel_data = [summary[c]['cancellation_delays'] for c in cancel_configs]
    cancel_labels = [format_config_label(c) for c in cancel_configs]

    if cancel_data:
        axes[0].boxplot(cancel_data, labels=cancel_labels)
        axes[0].set_ylabel('Cancellation Delay (ms)', fontsize=FONT_SIZE_LABEL)
        # axes[0].set_title('Cancellation Delay Distribution',
        #                   fontsize=FONT_SIZE_TITLE)
        axes[0].tick_params(axis='x', rotation=45,
                            labelsize=FONT_SIZE_TICK_LABELS)
        axes[0].tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
        axes[0].grid(True, alpha=0.3, axis='y')

    # Total runtime distribution
    runtime_data = [summary[c]['total_runtimes'] for c in configs]
    axes[1].boxplot(runtime_data, labels=labels)
    axes[1].set_ylabel('Total Runtime (ms)', fontsize=FONT_SIZE_LABEL)
    # axes[1].set_title('Total Runtime Distribution',
    #                   fontsize=FONT_SIZE_TITLE)
    axes[1].tick_params(axis='x', rotation=45, labelsize=FONT_SIZE_TICK_LABELS)
    axes[1].tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
    axes[1].grid(True, alpha=0.3, axis='y')

    # Layers processed distribution
    layers_data = [summary[c]['layers_processed'] for c in configs]
    axes[2].boxplot(layers_data, labels=labels)
    axes[2].set_ylabel('Layers Processed', fontsize=FONT_SIZE_LABEL)
    # axes[2].set_title('Layers Processed Distribution',
    #                   fontsize=FONT_SIZE_TITLE)
    axes[2].tick_params(axis='x', rotation=45, labelsize=FONT_SIZE_TICK_LABELS)
    axes[2].tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
    axes[2].grid(True, alpha=0.3, axis='y')
    if cancel_after_layers is not None:
        axes[2].axhline(y=cancel_after_layers, color='r', linestyle='--',
                        linewidth=LINE_WIDTH, label='Cancellation Threshold')
    # axes[2].legend(fontsize=LEGEND_SIZE)

    plt.tight_layout(pad=0)
    plt.savefig(PHASE4_DIR / 'distribution_boxplots.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()


def create_separate_legend():
    """
    Create a separate legend file for all plots
    """
    print("  - Creating separate legend")

    from matplotlib.patches import Patch

    # Define explicit colors matching the plots
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c',
              '#d62728']  # blue, orange, green, red

    # Create legend for the 4 proactive configuration combinations
    legend_elements = []
    for i, (sync, threading) in enumerate([
        ('sync', 'single'),
        ('sync', 'multi'),
        ('async', 'single'),
        ('async', 'multi'),
    ]):
        legend_elements.append(
            Patch(facecolor=colors[i], label=f'{sync}-{threading}')
        )

    # Create a minimal figure just for the legend (1 row, 4 columns)
    fig_legend = plt.figure(figsize=(PLOT_WIDTH, 0.5))
    legend = fig_legend.legend(
        handles=legend_elements,
        loc='center',
        ncol=4,
        fontsize=LEGEND_SIZE,
        frameon=False
    )

    # Save with minimal margins
    plt.savefig(PHASE4_DIR / 'legend.pdf', dpi=PLOT_DPI,
                bbox_inches='tight', pad_inches=0)
    plt.close()


def export_phase4_results(summary, cancel_after_layers=None, score_threshold=None):
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
            },
            'detections_at_cancel': {
                'mean': stats['avg_detections_at_cancel'],
                'std': stats['std_detections_at_cancel'],
            } if stats['avg_detections_at_cancel'] is not None else None,
            'population_breakdown': {
                'num_score_cancelled': stats['num_score_cancelled'],
                'num_deadline': stats['num_deadline'],
                'score_cancelled_avg_delay_ms': stats['avg_score_cancelled_delay_ms'],
                'score_cancelled_avg_runtime_ms': stats['avg_score_cancelled_runtime_ms'],
                'score_cancelled_avg_layers': stats['avg_score_cancelled_layers'],
                'deadline_avg_runtime_ms': stats['avg_deadline_runtime_ms'],
                'deadline_avg_layers': stats['avg_deadline_layers'],
            },
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
        f.write(f"  - Cancel after layers: {cancel_after_layers if cancel_after_layers is not None else 'N/A'}\n")
        f.write(f"  - Score threshold: {score_threshold if score_threshold is not None else 'N/A'}\n")
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

            if stats['avg_detections_at_cancel'] is not None:
                f.write(f"  Detections at Cancellation:\n")
                f.write(f"    Mean: {stats['avg_detections_at_cancel']:.2f}\n")
                f.write(f"    Std:  {stats['std_detections_at_cancel']:.2f}\n\n")

            f.write(f"  Population Breakdown:\n")
            f.write(f"    Score-cancelled: {stats['num_score_cancelled']} goals\n")
            f.write(f"    Hard-deadline:   {stats['num_deadline']} goals\n")
            if stats['avg_score_cancelled_delay_ms'] is not None:
                f.write(f"    Score-cancelled avg delay:   {stats['avg_score_cancelled_delay_ms']:.2f} ms\n")
                f.write(f"    Score-cancelled avg runtime: {stats['avg_score_cancelled_runtime_ms']:.2f} ms\n")
                f.write(f"    Score-cancelled avg layers:  {stats['avg_score_cancelled_layers']:.2f}\n")
            if stats['avg_deadline_runtime_ms'] is not None:
                f.write(f"    Hard-deadline avg runtime:   {stats['avg_deadline_runtime_ms']:.2f} ms\n")
                f.write(f"    Hard-deadline avg layers:    {stats['avg_deadline_layers']:.2f}\n")
            f.write("\n")

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
    print("STEP 7: CANCELLATION ANALYSIS")
    print("="*80)
    print(f"Input:  {TRACE_DIR}")
    print(f"Output: {PHASE4_DIR}")

    # Read client config for cancel_after_layers and score_threshold
    cancel_after_layers = None
    score_threshold = None
    client_config_path = CONFIG_DIR / "phase4_client.yaml"
    if client_config_path.exists():
        with open(client_config_path) as f:
            client_cfg = yaml.safe_load(f)
        cancel_after_layers = client_cfg['anytime_client']['ros__parameters']['cancel_after_layers']
        score_threshold = client_cfg['anytime_client']['ros__parameters']['score_threshold']
        print(f"Client config: cancel_after_layers={cancel_after_layers}, score_threshold={score_threshold}")
    else:
        print(f"WARNING: Client config not found at {client_config_path}, using no reference lines")

    # Find Phase 4 traces (only proactive architecture)
    phase4_traces = [d for d in TRACE_DIR.iterdir()
                     if d.is_dir() and 'phase4_' in d.name and 'proactive' in d.name]

    if not phase4_traces:
        print("\n❌ Error: No Phase 4 traces found!")
        print(f"Expected traces in: {TRACE_DIR}")
        print("Please run Step 6 first: ./6_run_experiments.sh")
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
    summary = aggregate_metrics(all_metrics, cancel_after_layers=cancel_after_layers)

    print(f"\nAggregated into {len(summary)} configurations")

    # Generate plots
    print("\nGenerating plots...")
    plot_cancellation_delay_comparison(summary)
    plot_total_runtime_comparison(summary)
    plot_layers_processed_comparison(summary, cancel_after_layers=cancel_after_layers)
    plot_metrics_by_block_size(summary)
    plot_distribution_boxplots(summary, cancel_after_layers=cancel_after_layers)
    create_separate_legend()

    print(f"  All plots saved to: {PHASE4_DIR}")

    # Export results
    export_phase4_results(summary, cancel_after_layers=cancel_after_layers, score_threshold=score_threshold)

    print("\n" + "="*80)
    print("✅ STEP 7 COMPLETE: CANCELLATION ANALYSIS")
    print("="*80)
    print(f"Results: {PHASE4_DIR}")
    print("\nGenerated outputs:")
    print("  - phase4_analysis.json (machine-readable)")
    print("  - phase4_summary.txt (human-readable)")
    print("  - cancellation_delay_comparison.pdf")
    print("  - total_runtime_comparison.pdf")
    print("  - layers_processed_comparison.pdf")
    print("  - metrics_by_block_size.pdf")
    print("  - distribution_boxplots.pdf")
    print("  - legend.pdf")
    print("\n🎉 All experiments complete!")


if __name__ == '__main__':
    main()
