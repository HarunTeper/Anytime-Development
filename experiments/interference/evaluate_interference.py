#!/usr/bin/env python3
"""
Interference Experiment Evaluation Script

This script parses LTTng traces from Interference experiments and generates:
- Primary Metrics: Timer period jitter (deviation from expected 100ms)
- Secondary Metrics: Monte Carlo compute batch timing
- Plots: Timer jitter vs batch size, mode comparison, threading comparison
- CSV/JSON exports of results
"""

import subprocess
import json
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from collections import defaultdict
import re
from datetime import datetime
from concurrent.futures import ProcessPoolExecutor

# Configuration
SCRIPT_DIR = Path(__file__).resolve().parent
EXPERIMENT_DIR = SCRIPT_DIR
TRACE_DIR = EXPERIMENT_DIR / "traces"
RESULTS_DIR = EXPERIMENT_DIR / "results"
PLOTS_DIR = RESULTS_DIR / "plots"

# Expected timer period (ms)
EXPECTED_TIMER_PERIOD_MS = 100.0

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
PLOTS_DIR.mkdir(exist_ok=True)


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
    Parse a single trace directory using babeltrace with filtering
    """
    print(f"  Parsing trace: {trace_dir.name}")

    # Use babeltrace2 to parse traces
    try:
        result = subprocess.run(
            ['babeltrace', str(trace_dir)],
            capture_output=True,
            text=True,
            check=True
        )
    except (subprocess.CalledProcessError, FileNotFoundError):
        print(f"    Error: Failed to parse trace with babeltrace")
        return []

    # Pre-filter lines to only process anytime events
    anytime_lines = [line for line in result.stdout.split(
        '\n') if 'anytime:' in line]

    events = []
    for line in anytime_lines:
        if not line.strip():
            continue

        try:
            # Parse timestamp (format: [HH:MM:SS.nanoseconds])
            timestamp_match = re.search(r'\[(\d+):(\d+):(\d+)\.(\d+)\]', line)
            if not timestamp_match:
                continue

            hours, minutes, seconds, nanoseconds = timestamp_match.groups()
            # Convert to total nanoseconds from start
            timestamp_ns = (int(hours) * 3600 + int(minutes) *
                            60 + int(seconds)) * 1_000_000_000 + int(nanoseconds)

            # Parse event name (format: provider:event_name:)
            event_match = re.search(r'anytime:([^:]+):', line)
            if not event_match:
                continue

            event_name = event_match.group(1)

            # Parse fields (everything after event name, inside braces)
            fields_match = re.search(r'\{(.+)\}', line)
            if not fields_match:
                fields = {}
            else:
                fields_str = fields_match.group(1)
                fields = {}

                # Parse key-value pairs
                for field in fields_str.split(','):
                    field = field.strip()
                    if '=' in field:
                        key, value = field.split('=', 1)
                        key = key.strip()
                        value = value.strip()

                        # Try to convert to appropriate type
                        if value.isdigit():
                            value = int(value)
                        elif value.replace('.', '', 1).isdigit():
                            value = float(value)
                        elif value.startswith('"') and value.endswith('"'):
                            value = value[1:-1]

                        fields[key] = value

            events.append(TraceEvent(timestamp_ns, event_name, fields))

        except Exception as e:
            # Skip malformed lines
            continue

    print(f"    Found {len(events)} anytime events")
    return events


def extract_metrics_from_events(events, config_name):
    """
    Extract metrics from a list of trace events

    Focus:
    - Timer period jitter (time between consecutive timer callback entries)
    - Monte Carlo compute batch timing

    Returns:
        Dictionary with metrics
    """
    # Parse batch_size from config_name
    config_parts = config_name.split('_')
    batch_size = int(config_parts[1])

    metrics = {
        'config': config_name,
        'batch_size': batch_size,

        # Timer metrics (PRIMARY FOCUS)
        'timer_periods': [],  # Time between consecutive timer starts (ms)
        'timer_jitter': [],   # Deviation from expected period (ms)
        'timer_execution_times': [],  # Time spent in timer callback (ms)
        'total_timer_callbacks': 0,
        'missed_periods': 0,  # Number of periods > 150% of expected

        # Monte Carlo compute metrics (SECONDARY)
        'compute_times': [],  # Time per compute batch (ms)
        'total_compute_batches': 0,
    }

    # Track state
    prev_timer_entry_time = None
    current_compute_entry_time = None

    for event in events:
        # === INTERFERENCE TIMER EVENTS (PRIMARY) ===
        if event.event_name == 'interference_timer_callback_entry':
            current_time = event.timestamp
            metrics['total_timer_callbacks'] += 1

            # Calculate period since last timer start
            if prev_timer_entry_time is not None:
                period_ns = current_time - prev_timer_entry_time
                period_ms = period_ns / 1_000_000.0
                metrics['timer_periods'].append(period_ms)

                # Calculate jitter (deviation from expected period)
                jitter_ms = period_ms - EXPECTED_TIMER_PERIOD_MS
                metrics['timer_jitter'].append(jitter_ms)

                # Check for missed periods (period > 150% of expected)
                if period_ms > (EXPECTED_TIMER_PERIOD_MS * 1.5):
                    metrics['missed_periods'] += 1

            prev_timer_entry_time = current_time

        elif event.event_name == 'interference_timer_callback_exit':
            # Calculate execution time
            if prev_timer_entry_time is not None:
                execution_ns = event.timestamp - prev_timer_entry_time
                execution_ms = execution_ns / 1_000_000.0
                metrics['timer_execution_times'].append(execution_ms)

        # === MONTE CARLO COMPUTE EVENTS (SECONDARY) ===
        elif event.event_name == 'anytime_compute_entry':
            current_compute_entry_time = event.timestamp

        elif event.event_name == 'anytime_compute_exit':
            if current_compute_entry_time is not None:
                compute_ns = event.timestamp - current_compute_entry_time
                compute_ms = compute_ns / 1_000_000.0
                metrics['compute_times'].append(compute_ms)
                metrics['total_compute_batches'] += 1
                current_compute_entry_time = None

    # Compute summary statistics for timer metrics
    if metrics['timer_periods']:
        metrics['avg_timer_period'] = np.mean(metrics['timer_periods'])
        metrics['std_timer_period'] = np.std(metrics['timer_periods'])
        metrics['min_timer_period'] = np.min(metrics['timer_periods'])
        metrics['max_timer_period'] = np.max(metrics['timer_periods'])
        metrics['median_timer_period'] = np.median(metrics['timer_periods'])
    else:
        metrics['avg_timer_period'] = 0
        metrics['std_timer_period'] = 0
        metrics['min_timer_period'] = 0
        metrics['max_timer_period'] = 0
        metrics['median_timer_period'] = 0

    if metrics['timer_jitter']:
        metrics['avg_jitter'] = np.mean(metrics['timer_jitter'])
        metrics['std_jitter'] = np.std(metrics['timer_jitter'])
        metrics['max_abs_jitter'] = np.max(np.abs(metrics['timer_jitter']))
    else:
        metrics['avg_jitter'] = 0
        metrics['std_jitter'] = 0
        metrics['max_abs_jitter'] = 0

    if metrics['timer_execution_times']:
        metrics['avg_timer_execution'] = np.mean(
            metrics['timer_execution_times'])
        metrics['std_timer_execution'] = np.std(
            metrics['timer_execution_times'])
    else:
        metrics['avg_timer_execution'] = 0
        metrics['std_timer_execution'] = 0

    # Compute summary statistics for compute metrics
    if metrics['compute_times']:
        metrics['avg_compute_time'] = np.mean(metrics['compute_times'])
        metrics['std_compute_time'] = np.std(metrics['compute_times'])
        metrics['min_compute_time'] = np.min(metrics['compute_times'])
        metrics['max_compute_time'] = np.max(metrics['compute_times'])
    else:
        metrics['avg_compute_time'] = 0
        metrics['std_compute_time'] = 0
        metrics['min_compute_time'] = 0
        metrics['max_compute_time'] = 0

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
        base_config = re.sub(r'_run\d+$', '', metrics['config'])
        config_groups[base_config].append(metrics)

    # Aggregate each group
    aggregated = {}
    for base_config, runs in config_groups.items():
        agg = {
            'config': base_config,
            'batch_size': runs[0]['batch_size'],
            'num_runs': len(runs),

            # Timer metrics - averages across runs
            'avg_timer_period': np.mean([r['avg_timer_period'] for r in runs]),
            'std_timer_period': np.mean([r['std_timer_period'] for r in runs]),
            'min_timer_period': np.min([r['min_timer_period'] for r in runs]),
            'max_timer_period': np.max([r['max_timer_period'] for r in runs]),
            'median_timer_period': np.mean([r['median_timer_period'] for r in runs]),

            'avg_jitter': np.mean([r['avg_jitter'] for r in runs]),
            'std_jitter': np.mean([r['std_jitter'] for r in runs]),
            'max_abs_jitter': np.max([r['max_abs_jitter'] for r in runs]),

            'avg_timer_execution': np.mean([r['avg_timer_execution'] for r in runs]),
            'std_timer_execution': np.mean([r['std_timer_execution'] for r in runs]),

            'total_timer_callbacks': np.mean([r['total_timer_callbacks'] for r in runs]),
            'missed_periods': np.mean([r['missed_periods'] for r in runs]),
            'missed_periods_percent': np.mean([r['missed_periods'] / r['total_timer_callbacks'] * 100.0
                                               if r['total_timer_callbacks'] > 0 else 0 for r in runs]),

            # Compute metrics
            'avg_compute_time': np.mean([r['avg_compute_time'] for r in runs]),
            'std_compute_time': np.mean([r['std_compute_time'] for r in runs]),
            'min_compute_time': np.min([r['min_compute_time'] for r in runs if r['min_compute_time'] > 0] or [0]),
            'max_compute_time': np.max([r['max_compute_time'] for r in runs]),
            'total_compute_batches': np.mean([r['total_compute_batches'] for r in runs]),
        }

        aggregated[base_config] = agg

    return aggregated


def parse_config_name(config_name):
    """Parse configuration name into components"""
    # Format: batch_<size>_<mode>_<threading>
    parts = config_name.split('_')

    # Skip 'test' prefix if present
    offset = 1 if parts[0] == 'test' else 0

    return {
        'batch_size': int(parts[1 + offset]),
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
    plt.style.use('seaborn-darkgrid')

    # Plot 1: Timer Period vs Batch Size
    print("  - Timer period vs batch size")
    fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT))

    # Get all unique batch sizes (reversed: high to low)
    all_batch_sizes = sorted(df['batch_size'].unique(), reverse=True)
    x = np.arange(len(all_batch_sizes))
    width = 0.35

    for i, mode in enumerate(['reactive', 'proactive']):
        threading = 'single'  # Only single-threaded
        offset = (i - 0.5) * width
        mask = (df['mode'] == mode) & (df['threading'] == threading)
        data = df[mask].sort_values('batch_size')

        if len(data) > 0:
            label = f"{mode.capitalize()}"
            # Align with all_batch_sizes
            y_values = [data[data['batch_size'] == bs]['avg_timer_period'].values[0]
                        if bs in data['batch_size'].values else 0
                        for bs in all_batch_sizes]
            y_errors = [data[data['batch_size'] == bs]['std_timer_period'].values[0]
                        if bs in data['batch_size'].values else 0
                        for bs in all_batch_sizes]

            ax.bar(x + offset, y_values, width, yerr=y_errors,
                   label=label, capsize=CAPSIZE)

    # Add expected period line
    ax.axhline(y=EXPECTED_TIMER_PERIOD_MS, color='red',
               linestyle=':', linewidth=LINE_WIDTH, label=f'Expected ({EXPECTED_TIMER_PERIOD_MS} ms)')

    ax.set_xlabel('Batch Size', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Average Timer Period (ms)', fontsize=FONT_SIZE_LABEL)
    # ax.set_title('Timer Period vs Batch Size\n(Deviation indicates interference)',
    #              fontsize=FONT_SIZE_TITLE, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(all_batch_sizes, fontsize=FONT_SIZE_TICK_LABELS)
    ax.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
    # ax.legend(fontsize=LEGEND_SIZE)
    ax.grid(True, axis='y', alpha=0.3)

    plt.tight_layout(pad=0)
    plt.savefig(PLOTS_DIR / 'timer_period_vs_batch_size.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()

    # Plot 2: Absolute Jitter vs Batch Size
    print("  - Jitter vs batch size")
    fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT))

    # Get all unique batch sizes (reversed: high to low)
    all_batch_sizes = sorted(df['batch_size'].unique(), reverse=True)
    x = np.arange(len(all_batch_sizes))
    width = 0.35

    for i, mode in enumerate(['reactive', 'proactive']):
        threading = 'single'  # Only single-threaded
        offset = (i - 0.5) * width
        mask = (df['mode'] == mode) & (df['threading'] == threading)
        data = df[mask].sort_values('batch_size')

        if len(data) > 0:
            label = f"{mode.capitalize()}"
            # Align with all_batch_sizes
            y_values = [data[data['batch_size'] == bs]['max_abs_jitter'].values[0]
                        if bs in data['batch_size'].values else 0
                        for bs in all_batch_sizes]
            y_errors = [data[data['batch_size'] == bs]['std_jitter'].values[0]
                        if bs in data['batch_size'].values else 0
                        for bs in all_batch_sizes]

            ax.bar(x + offset, y_values, width, yerr=y_errors,
                   label=label, capsize=CAPSIZE)

    ax.set_xlabel('Batch Size', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Maximum Absolute Jitter (ms)', fontsize=FONT_SIZE_LABEL)
    # ax.set_title('Timer Jitter vs Batch Size\n(Higher jitter = more interference)',
    #              fontsize=FONT_SIZE_TITLE, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(all_batch_sizes, fontsize=FONT_SIZE_TICK_LABELS)
    ax.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
    # ax.legend(fontsize=LEGEND_SIZE)
    ax.grid(True, axis='y', alpha=0.3)

    plt.tight_layout(pad=0)
    plt.savefig(PLOTS_DIR / 'jitter_vs_batch_size.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()

    # Plot 3: Missed Periods Percentage
    print("  - Missed periods analysis")
    fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT_SMALL))

    # Get all unique batch sizes (reversed: high to low)
    all_batch_sizes = sorted(df['batch_size'].unique(), reverse=True)
    x = np.arange(len(all_batch_sizes))
    width = 0.35

    for i, mode in enumerate(['reactive', 'proactive']):
        threading = 'single'  # Only single-threaded
        offset = (i - 0.5) * width
        mask = (df['mode'] == mode) & (df['threading'] == threading)
        data = df[mask].sort_values('batch_size')

        if len(data) > 0:
            label = f"{mode.capitalize()}"
            # Align with all_batch_sizes
            y_values = [data[data['batch_size'] == bs]['missed_periods_percent'].values[0]
                        if bs in data['batch_size'].values else 0
                        for bs in all_batch_sizes]

            ax.bar(x + offset, y_values, width, label=label)

    ax.set_xlabel('Batch Size', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Missed Periods (%)', fontsize=FONT_SIZE_LABEL)
    # ax.set_title('Percentage of Timer Periods Missed\n(Period > 150% of expected)',
    #              fontsize=FONT_SIZE_TITLE, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(all_batch_sizes, fontsize=FONT_SIZE_TICK_LABELS)
    ax.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
    # ax.legend(fontsize=LEGEND_SIZE)
    ax.grid(True, axis='y', alpha=0.3)

    plt.tight_layout(pad=0)
    plt.savefig(PLOTS_DIR / 'missed_periods_percentage.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()

    # Plot 4: Compute Time vs Batch Size
    print("  - Compute time vs batch size")
    fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT))

    # Get all unique batch sizes (reversed: high to low)
    all_batch_sizes = sorted(df['batch_size'].unique(), reverse=True)
    x = np.arange(len(all_batch_sizes))
    width = 0.35

    for i, mode in enumerate(['reactive', 'proactive']):
        threading = 'single'  # Only single-threaded
        offset = (i - 0.5) * width
        mask = (df['mode'] == mode) & (df['threading'] == threading)
        data = df[mask].sort_values('batch_size')

        if len(data) > 0:
            label = f"{mode.capitalize()}"
            # Align with all_batch_sizes
            y_values = [data[data['batch_size'] == bs]['avg_compute_time'].values[0]
                        if bs in data['batch_size'].values else 0
                        for bs in all_batch_sizes]
            y_errors = [data[data['batch_size'] == bs]['std_compute_time'].values[0]
                        if bs in data['batch_size'].values else 0
                        for bs in all_batch_sizes]

            ax.bar(x + offset, y_values, width, yerr=y_errors,
                   label=label, capsize=CAPSIZE)

    ax.set_xlabel('Batch Size', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Average Compute Batch Time (ms)', fontsize=FONT_SIZE_LABEL)
    # ax.set_title('Monte Carlo Compute Batch Time vs Batch Size',
    #              fontsize=FONT_SIZE_TITLE, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(all_batch_sizes, fontsize=FONT_SIZE_TICK_LABELS)
    ax.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
    # ax.legend(fontsize=LEGEND_SIZE)
    ax.grid(True, axis='y', alpha=0.3)

    plt.tight_layout(pad=0)
    plt.savefig(PLOTS_DIR / 'compute_time_vs_batch_size.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()

    # Plot 5: Timer Period Distribution (Box Plot)
    print("  - Timer period distribution")
    fig, axes = plt.subplots(1, 2, figsize=(16, 8))

    threading = 'single'  # Only single-threaded
    algorithm_modes = ['reactive', 'proactive']

    for j, mode in enumerate(algorithm_modes):
        ax = axes[j]

        mask = (df['mode'] == mode) & (df['threading'] == threading)
        data = df[mask].sort_values('batch_size')

        if len(data) > 0:
            # Create box plot data
            batch_labels = []
            period_data = []

            for _, row in data.iterrows():
                batch_labels.append(str(row['batch_size']))
                # Simulate distribution from mean and std
                # (In reality, you'd want to preserve raw period data)
                period_data.append([row['avg_timer_period']])

            bp = ax.boxplot(period_data, labels=batch_labels,
                            patch_artist=True, showmeans=True)

            # Color boxes
            for patch in bp['boxes']:
                patch.set_facecolor('lightblue')

            # Add expected period line
            ax.axhline(y=EXPECTED_TIMER_PERIOD_MS, color='red',
                       linestyle=':', linewidth=LINE_WIDTH, label='Expected')

        ax.set_xlabel('Batch Size', fontsize=FONT_SIZE_LABEL)
        ax.set_ylabel('Timer Period (ms)', fontsize=FONT_SIZE_LABEL)
        # ax.set_title(f'{mode.capitalize()}',
        #              fontsize=FONT_SIZE_TITLE, fontweight='bold')
        ax.tick_params(axis='both', labelsize=FONT_SIZE_TICK_LABELS)
        ax.grid(True, alpha=0.3)
        # ax.legend()

    # plt.suptitle('Timer Period Distribution by Configuration',
    #              fontsize=FONT_SIZE_TITLE, fontweight='bold')
    plt.tight_layout(pad=0)
    plt.savefig(PLOTS_DIR / 'timer_period_distribution.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()

    # Create separate legend file
    print("  - Creating separate legend")

    # Create legend entries with correct colors matching the plots
    from matplotlib.patches import Patch
    legend_elements = []
    for i, mode in enumerate(['reactive', 'proactive']):
        label = f"{mode.capitalize()}"
        legend_elements.append(Patch(facecolor=f'C{i}', label=label))

    # Add expected period line to legend
    legend_elements.append(plt.Line2D([0], [0], color='red', linestyle=':',
                                      linewidth=LINE_WIDTH, label=f'Expected ({EXPECTED_TIMER_PERIOD_MS} ms)'))

    # Create a minimal figure just for the legend
    fig_legend = plt.figure(figsize=(PLOT_WIDTH, 0.5))
    legend = fig_legend.legend(
        handles=legend_elements,
        loc='center',
        ncol=3,
        fontsize=LEGEND_SIZE,
        frameon=False
    )

    plt.savefig(PLOTS_DIR / 'legend.pdf', dpi=PLOT_DPI,
                bbox_inches='tight', pad_inches=0)
    plt.close()

    print(f"\nAll plots saved to: {PLOTS_DIR}")


def process_single_trace(trace_dir):
    """Process a single trace directory (for parallel processing)"""
    try:
        events = parse_trace_directory(trace_dir)
        if not events:
            return None

        metrics = extract_metrics_from_events(events, trace_dir.name)
        return metrics
    except Exception as e:
        print(f"  Error processing {trace_dir.name}: {e}")
        return None


def main():
    print("="*80)
    print("Interference Experiment Evaluation")
    print("="*80)
    print()

    # Find all trace directories
    trace_dirs = sorted([d for d in TRACE_DIR.iterdir() if d.is_dir()])

    if not trace_dirs:
        print(f"Error: No trace directories found in {TRACE_DIR}")
        print("Please run the experiment first: ./run_interference_experiments.sh")
        return

    print(f"Found {len(trace_dirs)} trace directories to process")
    print()

    # Parse traces (optionally in parallel for speed)
    print("Parsing traces...")
    all_metrics = []

    # Sequential processing (easier to debug)
    for trace_dir in trace_dirs:
        metrics = process_single_trace(trace_dir)
        if metrics:
            all_metrics.append(metrics)

    # Parallel processing (uncomment for faster processing)
    # with ProcessPoolExecutor(max_workers=4) as executor:
    #     results = executor.map(process_single_trace, trace_dirs)
    #     all_metrics = [m for m in results if m is not None]

    if not all_metrics:
        print("Error: No metrics extracted from traces")
        return

    print(f"\nSuccessfully parsed {len(all_metrics)} traces")

    # Save individual run results
    print("\nSaving individual run results...")
    individual_df = pd.DataFrame(all_metrics)

    # Select key columns for CSV export
    csv_columns = [
        'config', 'batch_size',
        'avg_timer_period', 'std_timer_period', 'min_timer_period',
        'max_timer_period', 'median_timer_period',
        'avg_jitter', 'std_jitter', 'max_abs_jitter',
        'avg_timer_execution', 'std_timer_execution',
        'total_timer_callbacks', 'missed_periods',
        'avg_compute_time', 'std_compute_time', 'total_compute_batches'
    ]

    individual_df[csv_columns].to_csv(
        RESULTS_DIR / 'individual_runs.csv', index=False)
    print(f"  Saved: {RESULTS_DIR / 'individual_runs.csv'}")

    # Aggregate results across runs
    print("\nAggregating results across runs...")
    aggregated = aggregate_runs(all_metrics)

    # Save aggregated results
    aggregated_df = pd.DataFrame(aggregated.values())
    aggregated_df.to_csv(RESULTS_DIR / 'aggregated_results.csv', index=False)
    print(f"  Saved: {RESULTS_DIR / 'aggregated_results.csv'}")

    # Save as JSON
    with open(RESULTS_DIR / 'aggregated_results.json', 'w') as f:
        json.dump(aggregated, f, indent=2, default=str)
    print(f"  Saved: {RESULTS_DIR / 'aggregated_results.json'}")

    # Generate plots
    generate_plots(aggregated)

    # Print summary statistics
    print("\n" + "="*80)
    print("Summary Statistics")
    print("="*80)

    for config, metrics in sorted(aggregated.items()):
        parsed = parse_config_name(config)
        print(f"\n{config}:")
        print(f"  Batch Size: {parsed['batch_size']}")
        print(f"  Mode: {parsed['mode']}")
        print(f"  Threading: {parsed['threading']}")
        print(f"  Timer Callbacks: {metrics['total_timer_callbacks']:.0f}")
        print(
            f"  Avg Timer Period: {metrics['avg_timer_period']:.2f} ms (expected: {EXPECTED_TIMER_PERIOD_MS} ms)")
        print(f"  Avg Jitter: {metrics['avg_jitter']:.2f} ms")
        print(f"  Max Abs Jitter: {metrics['max_abs_jitter']:.2f} ms")
        print(
            f"  Missed Periods: {metrics['missed_periods']:.0f} ({metrics['missed_periods_percent']:.1f}%)")
        print(
            f"  Avg Compute Time: {metrics['avg_compute_time']:.2f} ms")

    print("\n" + "="*80)
    print("Evaluation Complete!")
    print("="*80)
    print(f"\nResults saved to: {RESULTS_DIR}")
    print(f"Plots saved to: {PLOTS_DIR}")


if __name__ == "__main__":
    main()
