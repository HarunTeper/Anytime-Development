#!/usr/bin/env python3
"""
Interference RRT* Experiment Evaluation Script

This script parses LTTng traces from Interference experiments and generates:
- Primary Metrics: Timer period jitter (deviation from expected 100ms)
- Secondary Metrics: RRT* compute batch timing
- Plots: Timer jitter vs batch size, mode comparison, threading comparison
- CSV/JSON exports of results
"""

import subprocess
import json
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path
from collections import defaultdict
import re

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

    # Use babeltrace to parse traces
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
                        if value.startswith('"') and value.endswith('"'):
                            value = value[1:-1]
                        else:
                            try:
                                value = int(value, 0)  # base 0 handles 0x hex
                            except ValueError:
                                try:
                                    value = float(value)
                                except ValueError:
                                    pass

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
    - Compute batch timing

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
        'skipped_firings': 0,  # Number of timer firings skipped by ROS 2 catch-up

        # Compute metrics (SECONDARY)
        'compute_times': [],  # Time per compute batch (ms)
        'total_compute_batches': 0,
    }

    # Warm-up filtering: discard events before the first compute entry.
    # Early timer callbacks include ROS 2 startup overhead (executor setup,
    # DDS discovery, map loading) and are not representative of steady-state.
    warmup_cutoff_ns = None
    for event in events:
        if event.event_name == 'anytime_compute_entry':
            warmup_cutoff_ns = event.timestamp
            break

    if warmup_cutoff_ns is not None:
        events = [e for e in events if e.timestamp >= warmup_cutoff_ns]

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

                # Count skipped firings based on ROS 2 wall timer semantics.
                # The timer advances next_call_time in whole periods from the
                # original schedule. If N periods elapsed between consecutive
                # dispatches, N-1 firings were skipped (never dispatched).
                periods_elapsed = period_ms / EXPECTED_TIMER_PERIOD_MS
                skipped = max(0, round(periods_elapsed) - 1)
                metrics['skipped_firings'] += skipped

            prev_timer_entry_time = current_time

        elif event.event_name == 'interference_timer_callback_exit':
            # Calculate execution time
            if prev_timer_entry_time is not None:
                execution_ns = event.timestamp - prev_timer_entry_time
                execution_ms = execution_ns / 1_000_000.0
                metrics['timer_execution_times'].append(execution_ms)

        # === RRT* COMPUTE EVENTS (SECONDARY) ===
        elif event.event_name == 'anytime_compute_entry':
            # If a previous entry had no matching exit (cancelled mid-batch),
            # discard the stale entry rather than pairing it with a future exit.
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


def combined_std(runs, avg_key, std_key):
    """Combined std via law of total variance: sqrt(E[sigma_i^2] + Var(mu_i))"""
    if not runs:
        return 0
    means = [r[avg_key] for r in runs]
    stds = [r[std_key] for r in runs]
    return float(np.sqrt(np.mean([s**2 for s in stds]) + np.var(means)))


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
            'std_timer_period': combined_std(runs, 'avg_timer_period', 'std_timer_period'),
            'min_timer_period': np.min([r['min_timer_period'] for r in runs]),
            'max_timer_period': np.max([r['max_timer_period'] for r in runs]),
            'median_timer_period': np.mean([r['median_timer_period'] for r in runs]),

            'avg_jitter': np.mean([r['avg_jitter'] for r in runs]),
            'std_jitter': combined_std(runs, 'avg_jitter', 'std_jitter'),
            'max_abs_jitter': np.mean([r['max_abs_jitter'] for r in runs]),
            'max_abs_jitter_std': np.std([r['max_abs_jitter'] for r in runs]),

            'avg_timer_execution': np.mean([r['avg_timer_execution'] for r in runs]),
            'std_timer_execution': combined_std(runs, 'avg_timer_execution', 'std_timer_execution'),

            'total_timer_callbacks': np.mean([r['total_timer_callbacks'] for r in runs]),
            'skipped_firings': np.mean([r['skipped_firings'] for r in runs]),
            'skipped_firings_percent': np.mean([
                r['skipped_firings'] / (r['total_timer_callbacks'] + r['skipped_firings']) * 100.0
                if (r['total_timer_callbacks'] + r['skipped_firings']) > 0 else 0 for r in runs]),

            # Raw timer periods from all runs (for distribution plots)
            'all_timer_periods': [p for r in runs for p in r['timer_periods']],

            # Compute metrics
            'avg_compute_time': np.mean([r['avg_compute_time'] for r in runs]),
            'std_compute_time': combined_std(runs, 'avg_compute_time', 'std_compute_time'),
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
    try:
        plt.style.use('seaborn-v0_8-darkgrid')
    except OSError:
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
    ax.set_xticks(x)
    ax.set_xticklabels(all_batch_sizes, fontsize=FONT_SIZE_TICK_LABELS)
    ax.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
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
            y_errors = [data[data['batch_size'] == bs]['max_abs_jitter_std'].values[0]
                        if bs in data['batch_size'].values else 0
                        for bs in all_batch_sizes]

            ax.bar(x + offset, y_values, width, yerr=y_errors,
                   label=label, capsize=CAPSIZE)

    ax.set_xlabel('Batch Size', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Mean Max Absolute Jitter (ms)', fontsize=FONT_SIZE_LABEL)
    ax.set_xticks(x)
    ax.set_xticklabels(all_batch_sizes, fontsize=FONT_SIZE_TICK_LABELS)
    ax.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
    ax.grid(True, axis='y', alpha=0.3)

    plt.tight_layout(pad=0)
    plt.savefig(PLOTS_DIR / 'jitter_vs_batch_size.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()

    # Plot 3: Skipped Firings Percentage
    print("  - Skipped firings analysis")
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
            y_values = [data[data['batch_size'] == bs]['skipped_firings_percent'].values[0]
                        if bs in data['batch_size'].values else 0
                        for bs in all_batch_sizes]

            ax.bar(x + offset, y_values, width, label=label)

    ax.set_xlabel('Batch Size', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Skipped Firings (%)', fontsize=FONT_SIZE_LABEL)
    ax.set_xticks(x)
    ax.set_xticklabels(all_batch_sizes, fontsize=FONT_SIZE_TICK_LABELS)
    ax.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
    ax.grid(True, axis='y', alpha=0.3)

    plt.tight_layout(pad=0)
    plt.savefig(PLOTS_DIR / 'skipped_firings_percentage.pdf',
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
    ax.set_xticks(x)
    ax.set_xticklabels(all_batch_sizes, fontsize=FONT_SIZE_TICK_LABELS)
    ax.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
    ax.grid(True, axis='y', alpha=0.3)

    plt.tight_layout(pad=0)
    plt.savefig(PLOTS_DIR / 'compute_time_vs_batch_size.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()

    # Plot 5: Timer Period Distribution (Box Plot)
    # Uses raw timer period data propagated through aggregation
    print("  - Timer period distribution")
    fig, axes = plt.subplots(1, 2, figsize=(16, 8))

    threading = 'single'  # Only single-threaded
    algorithm_modes = ['reactive', 'proactive']

    for j, mode in enumerate(algorithm_modes):
        ax = axes[j]

        # Look up raw data from aggregated_metrics directly (not from df)
        batch_labels = []
        period_data = []

        for config, metrics in sorted(aggregated_metrics.items(),
                                       key=lambda x: x[1]['batch_size']):
            parsed = parse_config_name(config)
            if parsed['mode'] != mode or parsed['threading'] != threading:
                continue
            raw_periods = metrics.get('all_timer_periods', [])
            if raw_periods:
                batch_labels.append(str(parsed['batch_size']))
                period_data.append(raw_periods)

        if period_data:
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
        ax.tick_params(axis='both', labelsize=FONT_SIZE_TICK_LABELS)
        ax.grid(True, alpha=0.3)

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


def run_statistical_tests(all_metrics, results_dir):
    """Run Mann-Whitney U tests comparing reactive vs proactive timer jitter."""
    from scipy.stats import mannwhitneyu

    results = []

    # Group raw timer periods by (batch_size, mode)
    groups = defaultdict(list)
    for m in all_metrics:
        base_config = re.sub(r'_run\d+$', '', m['config'])
        parsed = parse_config_name(base_config)
        key = (parsed['batch_size'], parsed['mode'])
        groups[key].extend(m['timer_periods'])

    # Test reactive vs proactive for each batch size
    for batch_size in sorted(set(k[0] for k in groups)):
        reactive_key = (batch_size, 'reactive')
        proactive_key = (batch_size, 'proactive')
        if reactive_key in groups and proactive_key in groups:
            r_data = groups[reactive_key]
            p_data = groups[proactive_key]
            if len(r_data) >= 5 and len(p_data) >= 5:
                stat, pval = mannwhitneyu(r_data, p_data, alternative='two-sided')
                results.append({
                    'comparison': 'reactive_vs_proactive',
                    'batch_size': batch_size,
                    'n_reactive': len(r_data),
                    'n_proactive': len(p_data),
                    'U_statistic': stat,
                    'p_value': pval,
                    'significant_005': pval < 0.05,
                })

    if results:
        stat_df = pd.DataFrame(results)
        stat_df.to_csv(results_dir / 'statistical_tests.csv', index=False)
        print(f"  Statistical tests saved to: {results_dir / 'statistical_tests.csv'}")

    return results


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
    print("Interference RRT* Experiment Evaluation")
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
        'total_timer_callbacks', 'skipped_firings',
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

    # Save condensed Table I: skipped firings % by mode and batch size
    table_batch_sizes = [1, 16, 64, 256, 1024, 4096, 16384]
    table_rows = {}
    for config, metrics in aggregated.items():
        parsed = parse_config_name(config)
        mode = parsed['mode'].capitalize()
        bs = parsed['batch_size']
        if bs in table_batch_sizes:
            table_rows.setdefault(mode, {})[bs] = metrics['skipped_firings_percent']
    table1_rows = []
    for mode in ['Proactive', 'Reactive']:
        if mode in table_rows:
            row = {'Configuration': mode}
            for bs in table_batch_sizes:
                row[str(bs)] = f"{table_rows[mode].get(bs, 0):.2f}%"
            table1_rows.append(row)
    table1_df = pd.DataFrame(table1_rows)
    table1_df.to_csv(RESULTS_DIR / 'table_1_skipped_firings.csv', index=False)
    print(f"  Saved: {RESULTS_DIR / 'table_1_skipped_firings.csv'}")

    # Run statistical significance tests
    print("\nRunning statistical significance tests...")
    try:
        stat_results = run_statistical_tests(all_metrics, RESULTS_DIR)
        sig_count = sum(1 for r in stat_results if r.get('significant_005'))
        print(f"  {len(stat_results)} tests performed, {sig_count} significant at p < 0.05")
    except ImportError:
        print("  WARNING: scipy not installed, skipping statistical tests")
        print("  Install with: pip3 install scipy")

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
            f"  Skipped Firings: {metrics['skipped_firings']:.0f} ({metrics['skipped_firings_percent']:.1f}%)")
        print(
            f"  Avg Compute Time: {metrics['avg_compute_time']:.2f} ms")

    print("\n" + "="*80)
    print("Evaluation Complete!")
    print("="*80)
    print(f"\nResults saved to: {RESULTS_DIR}")
    print(f"Plots saved to: {PLOTS_DIR}")


if __name__ == "__main__":
    main()
