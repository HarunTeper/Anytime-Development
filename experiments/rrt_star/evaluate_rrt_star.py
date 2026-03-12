#!/usr/bin/env python3
"""
RRT* Experiment Evaluation Script

This script parses LTTng traces from RRT* experiments and generates:
- Framework metrics: time per batch, throughput, cancellation delay, latencies
- Overhead metrics: per-batch overhead, overhead ratio, feedback/result times
- RRT*-specific metrics: convergence curves, first solution iteration, tree size
- CSV/JSON exports and PDF plots
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
from matplotlib.patches import Patch

# Configuration
SCRIPT_DIR = Path(__file__).resolve().parent
EXPERIMENT_DIR = SCRIPT_DIR
TRACE_DIR = EXPERIMENT_DIR / "traces"
RESULTS_DIR = EXPERIMENT_DIR / "results"
PLOTS_DIR = RESULTS_DIR / "plots"
CONVERGENCE_DIR = RESULTS_DIR / "convergence_data"

# Plot configuration
PLOT_WIDTH = 12
PLOT_HEIGHT = 8
PLOT_DPI = 300
FONT_SIZE_LABEL = 30
FONT_SIZE_TICK_LABELS = 30
FONT_SIZE_OFFSET = 30
LEGEND_SIZE = 30
CAPSIZE = 5

# Create output directories
RESULTS_DIR.mkdir(exist_ok=True)
PLOTS_DIR.mkdir(exist_ok=True)
CONVERGENCE_DIR.mkdir(exist_ok=True)


class TraceEvent:
    """Represents a single trace event"""

    def __init__(self, timestamp, event_name, fields):
        self.timestamp = timestamp
        self.event_name = event_name
        self.fields = fields

    def __repr__(self):
        return f"TraceEvent({self.timestamp}, {self.event_name})"


def parse_trace_directory(trace_dir):
    """Parse a single trace directory using babeltrace"""
    print(f"  Parsing trace: {trace_dir.name}")

    try:
        result = subprocess.run(
            ['babeltrace2', '--names', 'none', str(trace_dir)],
            capture_output=True, text=True, check=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        try:
            result = subprocess.run(
                ['babeltrace', str(trace_dir)],
                capture_output=True, text=True, check=True)
        except (subprocess.CalledProcessError, FileNotFoundError):
            print(f"    Error: babeltrace not found or failed")
            return []

    anytime_lines = [line for line in result.stdout.split('\n')
                     if 'anytime:' in line]

    events = []
    for line in anytime_lines:
        if not line.strip():
            continue

        try:
            ts_start = line.find('[')
            ts_end = line.find(']')
            if ts_start == -1 or ts_end == -1:
                continue

            timestamp_str = line[ts_start+1:ts_end]
            ts_parts = timestamp_str.split(':')
            timestamp = (int(ts_parts[0]) * 3600 + int(ts_parts[1]) * 60
                         + float(ts_parts[2])) * 1e9

            event_start = line.find('anytime:')
            if event_start == -1:
                continue
            event_part = line[event_start:]
            event_end = event_part.find(':', len('anytime:'))
            if event_end == -1:
                continue
            event_name = event_part[:event_end]

            fields = {}
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

            if brace_groups:
                fields_str = brace_groups[-1]
                for field_pair in fields_str.split(','):
                    if '=' in field_pair:
                        key, value = field_pair.split('=', 1)
                        fields[key.strip()] = value.strip()

            events.append(TraceEvent(timestamp, event_name, fields))

        except Exception:
            continue

    print(f"    Found {len(events)} anytime events")
    return events


def extract_metrics_from_events(events, config_name):
    """Extract all metrics from trace events"""
    config_parts = config_name.split('_')
    batch_size = int(config_parts[1])

    metrics = {
        'config': config_name,
        'batch_size': batch_size,
        'total_batches': 0,
        'total_iterations': 0,
        'batch_times': [],
        'cancellation_delays': [],
        'goal_to_finish_latencies': [],
        'goal_to_cancel_latencies': [],
        'cancel_to_finish_latencies': [],
        # Overhead metrics
        'per_batch_overheads': [],
        'overhead_ratios': [],
        'feedback_send_times': [],
        'result_compute_times': [],
        # RRT*-specific
        'convergence_data': [],
        'final_best_costs': [],
        'final_tree_sizes': [],
        'first_solution_iterations': [],
    }

    # State tracking
    current_compute_start = None
    prev_compute_exit = None
    current_feedback_start = None
    current_result_start = None
    cancel_request_time = None
    goal_sent_time = None
    cancel_sent_time = None

    for event in events:
        name = event.event_name

        if name == 'anytime:anytime_compute_entry':
            current_compute_start = event.timestamp
            metrics['total_batches'] += 1
            # Per-batch overhead: gap from previous compute_exit to this compute_entry
            if prev_compute_exit is not None:
                overhead_ms = (event.timestamp - prev_compute_exit) / 1e6
                metrics['per_batch_overheads'].append(overhead_ms)

        elif name == 'anytime:anytime_compute_exit':
            if current_compute_start is not None:
                batch_time_ms = (event.timestamp - current_compute_start) / 1e6
                metrics['batch_times'].append(batch_time_ms)
                # Compute overhead ratio for the previous gap
                if metrics['per_batch_overheads'] and len(metrics['per_batch_overheads']) == len(metrics['batch_times']):
                    overhead = metrics['per_batch_overheads'][-1]
                    ratio = overhead / (overhead + batch_time_ms) * 100.0 if (overhead + batch_time_ms) > 0 else 0
                    metrics['overhead_ratios'].append(ratio)
                prev_compute_exit = event.timestamp
                current_compute_start = None

        elif name == 'anytime:anytime_send_feedback_entry':
            current_feedback_start = event.timestamp

        elif name == 'anytime:anytime_send_feedback_exit':
            if current_feedback_start is not None:
                feedback_ms = (event.timestamp - current_feedback_start) / 1e6
                metrics['feedback_send_times'].append(feedback_ms)
                current_feedback_start = None

        elif name == 'anytime:anytime_calculate_result_entry':
            current_result_start = event.timestamp

        elif name == 'anytime:anytime_calculate_result_exit':
            if current_result_start is not None:
                result_ms = (event.timestamp - current_result_start) / 1e6
                metrics['result_compute_times'].append(result_ms)
                current_result_start = None

        elif name == 'anytime:anytime_server_handle_cancel':
            cancel_request_time = event.timestamp

        elif name == 'anytime:anytime_base_deactivate':
            if cancel_request_time is not None:
                delay_ms = (event.timestamp - cancel_request_time) / 1e6
                metrics['cancellation_delays'].append(delay_ms)
                cancel_request_time = None

        elif name == 'anytime:anytime_client_goal_sent':
            goal_sent_time = event.timestamp

        elif name == 'anytime:anytime_client_cancel_sent':
            cancel_sent_time = event.timestamp
            if goal_sent_time is not None:
                metrics['goal_to_cancel_latencies'].append(
                    (cancel_sent_time - goal_sent_time) / 1e6)

        elif name == 'anytime:anytime_client_goal_finished':
            finished_time = event.timestamp
            if goal_sent_time is not None:
                metrics['goal_to_finish_latencies'].append(
                    (finished_time - goal_sent_time) / 1e6)
            if cancel_sent_time is not None:
                metrics['cancel_to_finish_latencies'].append(
                    (finished_time - cancel_sent_time) / 1e6)
            goal_sent_time = None
            cancel_sent_time = None

        # RRT*-specific events
        elif name == 'anytime:rrt_star_iteration':
            try:
                iteration = int(event.fields.get('iteration_num', 0))
                tree_size = int(event.fields.get('tree_size', 0))
                best_cost = float(event.fields.get('best_cost', 'inf'))
                metrics['convergence_data'].append(
                    (iteration, best_cost, tree_size))
            except (ValueError, TypeError):
                pass

        elif name == 'anytime:rrt_star_result':
            try:
                best_cost = float(event.fields.get('best_cost', 'inf'))
                tree_size = int(event.fields.get('tree_size', 0))
                total_iters = int(event.fields.get('total_iterations', 0))
                metrics['final_best_costs'].append(best_cost)
                metrics['final_tree_sizes'].append(tree_size)
            except (ValueError, TypeError):
                pass

    # Summary statistics
    metrics['total_iterations'] = metrics['total_batches'] * batch_size

    def safe_mean(lst):
        return np.mean(lst) if lst else 0

    def safe_std(lst):
        return np.std(lst) if lst else 0

    metrics['avg_time_per_batch'] = safe_mean(metrics['batch_times'])
    metrics['std_time_per_batch'] = safe_std(metrics['batch_times'])
    metrics['avg_cancellation_delay'] = safe_mean(metrics['cancellation_delays'])
    metrics['std_cancellation_delay'] = safe_std(metrics['cancellation_delays'])
    metrics['avg_goal_to_finish_latency'] = safe_mean(metrics['goal_to_finish_latencies'])
    metrics['std_goal_to_finish_latency'] = safe_std(metrics['goal_to_finish_latencies'])
    metrics['avg_goal_to_cancel_latency'] = safe_mean(metrics['goal_to_cancel_latencies'])
    metrics['std_goal_to_cancel_latency'] = safe_std(metrics['goal_to_cancel_latencies'])
    metrics['avg_cancel_to_finish_latency'] = safe_mean(metrics['cancel_to_finish_latencies'])
    metrics['std_cancel_to_finish_latency'] = safe_std(metrics['cancel_to_finish_latencies'])

    # Overhead summary statistics
    metrics['avg_per_batch_overhead'] = safe_mean(metrics['per_batch_overheads'])
    metrics['std_per_batch_overhead'] = safe_std(metrics['per_batch_overheads'])
    metrics['avg_overhead_ratio'] = safe_mean(metrics['overhead_ratios'])
    metrics['std_overhead_ratio'] = safe_std(metrics['overhead_ratios'])
    metrics['avg_feedback_send_time'] = safe_mean(metrics['feedback_send_times'])
    metrics['std_feedback_send_time'] = safe_std(metrics['feedback_send_times'])
    metrics['avg_result_compute_time'] = safe_mean(metrics['result_compute_times'])
    metrics['std_result_compute_time'] = safe_std(metrics['result_compute_times'])

    # Batch time percentiles
    if metrics['batch_times']:
        metrics['batch_time_p50'] = np.percentile(metrics['batch_times'], 50)
        metrics['batch_time_p95'] = np.percentile(metrics['batch_times'], 95)
        metrics['batch_time_p99'] = np.percentile(metrics['batch_times'], 99)
    else:
        metrics['batch_time_p50'] = 0
        metrics['batch_time_p95'] = 0
        metrics['batch_time_p99'] = 0

    # RRT*-specific summary
    metrics['avg_final_best_cost'] = safe_mean(metrics['final_best_costs'])
    metrics['avg_final_tree_size'] = safe_mean(metrics['final_tree_sizes'])

    return metrics


def aggregate_runs(all_metrics):
    """Aggregate metrics from multiple runs of the same configuration"""
    config_groups = defaultdict(list)
    for metrics in all_metrics:
        base_config = metrics['config'].rsplit('_run', 1)[0]
        config_groups[base_config].append(metrics)

    aggregated = {}
    for base_config, runs in config_groups.items():
        agg = {
            'config': base_config,
            'num_runs': len(runs),
            'total_batches': np.mean([r['total_batches'] for r in runs]),
            'total_iterations': np.mean([r['total_iterations'] for r in runs]),
            'avg_time_per_batch': np.mean([r['avg_time_per_batch'] for r in runs]),
            'std_time_per_batch': np.mean([r['std_time_per_batch'] for r in runs]),
            'avg_cancellation_delay': np.mean([r['avg_cancellation_delay'] for r in runs if r['avg_cancellation_delay'] > 0]) if any(r['avg_cancellation_delay'] > 0 for r in runs) else 0,
            'std_cancellation_delay': np.mean([r['std_cancellation_delay'] for r in runs if r['std_cancellation_delay'] > 0]) if any(r['std_cancellation_delay'] > 0 for r in runs) else 0,
            'avg_goal_to_finish_latency': np.mean([r['avg_goal_to_finish_latency'] for r in runs if r['avg_goal_to_finish_latency'] > 0]) if any(r['avg_goal_to_finish_latency'] > 0 for r in runs) else 0,
            'std_goal_to_finish_latency': np.mean([r['std_goal_to_finish_latency'] for r in runs if r['std_goal_to_finish_latency'] > 0]) if any(r['std_goal_to_finish_latency'] > 0 for r in runs) else 0,
            'avg_goal_to_cancel_latency': np.mean([r['avg_goal_to_cancel_latency'] for r in runs if r['avg_goal_to_cancel_latency'] > 0]) if any(r['avg_goal_to_cancel_latency'] > 0 for r in runs) else 0,
            'std_goal_to_cancel_latency': np.mean([r['std_goal_to_cancel_latency'] for r in runs if r['std_goal_to_cancel_latency'] > 0]) if any(r['std_goal_to_cancel_latency'] > 0 for r in runs) else 0,
            'avg_cancel_to_finish_latency': np.mean([r['avg_cancel_to_finish_latency'] for r in runs if r['avg_cancel_to_finish_latency'] > 0]) if any(r['avg_cancel_to_finish_latency'] > 0 for r in runs) else 0,
            'std_cancel_to_finish_latency': np.mean([r['std_cancel_to_finish_latency'] for r in runs if r['std_cancel_to_finish_latency'] > 0]) if any(r['std_cancel_to_finish_latency'] > 0 for r in runs) else 0,
            # Overhead metrics
            'avg_per_batch_overhead': np.mean([r['avg_per_batch_overhead'] for r in runs]),
            'std_per_batch_overhead': np.mean([r['std_per_batch_overhead'] for r in runs]),
            'avg_overhead_ratio': np.mean([r['avg_overhead_ratio'] for r in runs]),
            'std_overhead_ratio': np.mean([r['std_overhead_ratio'] for r in runs]),
            'avg_feedback_send_time': np.mean([r['avg_feedback_send_time'] for r in runs]),
            'std_feedback_send_time': np.mean([r['std_feedback_send_time'] for r in runs]),
            'avg_result_compute_time': np.mean([r['avg_result_compute_time'] for r in runs]),
            'std_result_compute_time': np.mean([r['std_result_compute_time'] for r in runs]),
            'batch_time_p50': np.mean([r['batch_time_p50'] for r in runs]),
            'batch_time_p95': np.mean([r['batch_time_p95'] for r in runs]),
            'batch_time_p99': np.mean([r['batch_time_p99'] for r in runs]),
            # RRT*-specific
            'avg_final_best_cost': np.mean([r['avg_final_best_cost'] for r in runs if r['avg_final_best_cost'] > 0]) if any(r['avg_final_best_cost'] > 0 for r in runs) else 0,
            'avg_final_tree_size': np.mean([r['avg_final_tree_size'] for r in runs if r['avg_final_tree_size'] > 0]) if any(r['avg_final_tree_size'] > 0 for r in runs) else 0,
        }
        aggregated[base_config] = agg

    return aggregated


def parse_config_name(config_name):
    """Parse configuration name into components"""
    # Format: batch_<size>_<mode>_<threading>_<map>
    parts = config_name.split('_')
    offset = 1 if parts[0] == 'test' else 0

    return {
        'batch_size': int(parts[1 + offset]),
        'mode': parts[2 + offset],
        'threading': parts[3 + offset],
        'map': parts[4 + offset] if len(parts) > 4 + offset else 'unknown',
    }


def make_grouped_bar_chart(df, y_col, ylabel, filename, yerr_col=None,
                           map_filter=None, title_suffix=""):
    """Helper to create a grouped bar chart"""
    if map_filter:
        plot_df = df[df['map'] == map_filter]
        suffix = f" ({map_filter})" + title_suffix
    else:
        plot_df = df
        suffix = title_suffix

    if plot_df.empty:
        return

    fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT))
    all_batch_sizes = sorted(plot_df['batch_size'].unique())
    x = np.arange(len(all_batch_sizes))
    width = 0.2

    for i, mode in enumerate(['reactive', 'proactive']):
        for j, threading in enumerate(['single', 'multi']):
            data = plot_df[(plot_df['mode'] == mode) & (plot_df['threading'] == threading)]
            data = data.sort_values('batch_size')

            if data.empty:
                continue

            y_values = []
            yerr_values = []
            x_positions = []
            for bs in all_batch_sizes:
                bd = data[data['batch_size'] == bs]
                if not bd.empty:
                    y_values.append(bd[y_col].iloc[0])
                    if yerr_col and yerr_col in bd.columns:
                        yerr_values.append(bd[yerr_col].iloc[0])
                    x_positions.append(all_batch_sizes.index(bs))

            if y_values:
                offset = (i * 2 + j - 1.5) * width
                kwargs = {'label': f'{mode}-{threading}'}
                if yerr_values:
                    kwargs['yerr'] = yerr_values
                    kwargs['capsize'] = CAPSIZE
                ax.bar(np.array(x_positions) + offset, y_values, width, **kwargs)

    ax.set_xlabel('Batch Size', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel(ylabel, fontsize=FONT_SIZE_LABEL)
    ax.set_xticks(x)
    ax.set_xticklabels(all_batch_sizes, fontsize=FONT_SIZE_TICK_LABELS)
    ax.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
    ax.yaxis.get_offset_text().set_fontsize(FONT_SIZE_OFFSET)
    ax.grid(True, axis='y')

    plt.tight_layout(pad=0)
    plt.savefig(PLOTS_DIR / filename, dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()


def generate_plots(aggregated_metrics, all_metrics):
    """Generate all plots from aggregated metrics"""
    print("\nGenerating plots...")

    df = pd.DataFrame([
        {**parse_config_name(config), **metrics}
        for config, metrics in aggregated_metrics.items()
    ])

    plt.style.use('seaborn-darkgrid')

    # Get unique maps
    all_maps = sorted(df['map'].unique()) if 'map' in df.columns else ['unknown']

    # ==================== Framework Metrics ====================

    # 1. Batch size vs time per batch
    print("  - Batch size vs time per batch")
    for map_name in all_maps:
        make_grouped_bar_chart(df, 'avg_time_per_batch',
                               'Average Time per Batch (ms)',
                               f'batch_size_vs_time_{map_name}.pdf',
                               yerr_col='std_time_per_batch',
                               map_filter=map_name)
    make_grouped_bar_chart(df, 'avg_time_per_batch',
                           'Average Time per Batch (ms)',
                           'batch_size_vs_time.pdf',
                           yerr_col='std_time_per_batch')

    # 2. Throughput
    print("  - Throughput")
    df['throughput'] = df['batch_size'] / (df['avg_time_per_batch'] / 1000.0)
    df.loc[df['avg_time_per_batch'] == 0, 'throughput'] = 0
    for map_name in all_maps:
        make_grouped_bar_chart(df, 'throughput',
                               'Throughput (iterations/second)',
                               f'throughput_{map_name}.pdf',
                               map_filter=map_name)
    make_grouped_bar_chart(df, 'throughput',
                           'Throughput (iterations/second)',
                           'throughput.pdf')

    # 3. Cancellation delay
    print("  - Cancellation delay")
    for map_name in all_maps:
        make_grouped_bar_chart(df, 'avg_cancellation_delay',
                               'Average Cancellation Delay (ms)',
                               f'cancellation_delay_{map_name}.pdf',
                               map_filter=map_name)
    make_grouped_bar_chart(df, 'avg_cancellation_delay',
                           'Average Cancellation Delay (ms)',
                           'cancellation_delay.pdf')

    # 4. Goal-to-finish latency
    print("  - Goal-to-finish latency")
    make_grouped_bar_chart(df, 'avg_goal_to_finish_latency',
                           'Average Goal-to-Finish Latency (ms)',
                           'goal_to_finish_latency.pdf',
                           yerr_col='std_goal_to_finish_latency')

    # 5. Goal-to-cancel latency
    print("  - Goal-to-cancel latency")
    make_grouped_bar_chart(df, 'avg_goal_to_cancel_latency',
                           'Average Goal-to-Cancel Latency (ms)',
                           'goal_to_cancel_latency.pdf',
                           yerr_col='std_goal_to_cancel_latency')

    # 6. Cancel-to-finish latency
    print("  - Cancel-to-finish latency")
    make_grouped_bar_chart(df, 'avg_cancel_to_finish_latency',
                           'Average Cancel-to-Finish Latency (ms)',
                           'cancel_to_finish_latency.pdf',
                           yerr_col='std_cancel_to_finish_latency')

    # 7. Total iterations
    print("  - Total iterations")
    make_grouped_bar_chart(df, 'total_iterations',
                           'Total Iterations Completed',
                           'total_iterations.pdf')

    # 8. Total cancellation time
    print("  - Total cancellation time")
    df['total_cancellation_time'] = df['avg_goal_to_cancel_latency'] + df['avg_cancellation_delay']
    make_grouped_bar_chart(df, 'total_cancellation_time',
                           'Total Cancellation Time (ms)',
                           'total_cancellation_time.pdf')

    # ==================== Overhead Metrics ====================

    # 15. Per-batch overhead
    print("  - Per-batch overhead")
    for map_name in all_maps:
        make_grouped_bar_chart(df, 'avg_per_batch_overhead',
                               'Per-Batch Overhead (ms)',
                               f'per_batch_overhead_{map_name}.pdf',
                               yerr_col='std_per_batch_overhead',
                               map_filter=map_name)
    make_grouped_bar_chart(df, 'avg_per_batch_overhead',
                           'Per-Batch Overhead (ms)',
                           'per_batch_overhead.pdf',
                           yerr_col='std_per_batch_overhead')

    # 16. Overhead ratio
    print("  - Overhead ratio")
    for map_name in all_maps:
        make_grouped_bar_chart(df, 'avg_overhead_ratio',
                               'Overhead Ratio (%)',
                               f'overhead_ratio_{map_name}.pdf',
                               yerr_col='std_overhead_ratio',
                               map_filter=map_name)
    make_grouped_bar_chart(df, 'avg_overhead_ratio',
                           'Overhead Ratio (%)',
                           'overhead_ratio.pdf',
                           yerr_col='std_overhead_ratio')

    # 17. Feedback send time
    print("  - Feedback send time")
    for map_name in all_maps:
        make_grouped_bar_chart(df, 'avg_feedback_send_time',
                               'Feedback Send Time (ms)',
                               f'feedback_send_time_{map_name}.pdf',
                               yerr_col='std_feedback_send_time',
                               map_filter=map_name)
    make_grouped_bar_chart(df, 'avg_feedback_send_time',
                           'Feedback Send Time (ms)',
                           'feedback_send_time.pdf',
                           yerr_col='std_feedback_send_time')

    # 18. Result compute time
    print("  - Result compute time")
    for map_name in all_maps:
        make_grouped_bar_chart(df, 'avg_result_compute_time',
                               'Result Compute Time (ms)',
                               f'result_compute_time_{map_name}.pdf',
                               yerr_col='std_result_compute_time',
                               map_filter=map_name)
    make_grouped_bar_chart(df, 'avg_result_compute_time',
                           'Result Compute Time (ms)',
                           'result_compute_time.pdf',
                           yerr_col='std_result_compute_time')

    # 19. Batch time percentiles
    print("  - Batch time percentiles")
    for map_name in all_maps:
        plot_df = df[df['map'] == map_name] if map_name != 'unknown' else df
        if plot_df.empty:
            continue
        fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT))
        all_batch_sizes = sorted(plot_df['batch_size'].unique())
        x = np.arange(len(all_batch_sizes))
        width = 0.25

        for idx, (pct_col, pct_label) in enumerate([
            ('batch_time_p50', 'p50'), ('batch_time_p95', 'p95'), ('batch_time_p99', 'p99')
        ]):
            # Average across mode/threading for simplicity
            y_values = []
            for bs in all_batch_sizes:
                bd = plot_df[plot_df['batch_size'] == bs]
                y_values.append(bd[pct_col].mean() if not bd.empty else 0)
            offset = (idx - 1) * width
            ax.bar(x + offset, y_values, width, label=pct_label)

        ax.set_xlabel('Batch Size', fontsize=FONT_SIZE_LABEL)
        ax.set_ylabel('Batch Time (ms)', fontsize=FONT_SIZE_LABEL)
        ax.set_xticks(x)
        ax.set_xticklabels(all_batch_sizes, fontsize=FONT_SIZE_TICK_LABELS)
        ax.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
        ax.yaxis.get_offset_text().set_fontsize(FONT_SIZE_OFFSET)
        ax.legend(fontsize=LEGEND_SIZE)
        ax.grid(True, axis='y')
        plt.tight_layout(pad=0)
        plt.savefig(PLOTS_DIR / f'batch_time_percentiles_{map_name}.pdf',
                    dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
        plt.close()

    # Also combined
    fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT))
    all_batch_sizes = sorted(df['batch_size'].unique())
    x = np.arange(len(all_batch_sizes))
    width = 0.25
    for idx, (pct_col, pct_label) in enumerate([
        ('batch_time_p50', 'p50'), ('batch_time_p95', 'p95'), ('batch_time_p99', 'p99')
    ]):
        y_values = []
        for bs in all_batch_sizes:
            bd = df[df['batch_size'] == bs]
            y_values.append(bd[pct_col].mean() if not bd.empty else 0)
        offset = (idx - 1) * width
        ax.bar(x + offset, y_values, width, label=pct_label)

    ax.set_xlabel('Batch Size', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Batch Time (ms)', fontsize=FONT_SIZE_LABEL)
    ax.set_xticks(x)
    ax.set_xticklabels(all_batch_sizes, fontsize=FONT_SIZE_TICK_LABELS)
    ax.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
    ax.yaxis.get_offset_text().set_fontsize(FONT_SIZE_OFFSET)
    ax.legend(fontsize=LEGEND_SIZE)
    ax.grid(True, axis='y')
    plt.tight_layout(pad=0)
    plt.savefig(PLOTS_DIR / 'batch_time_percentiles.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()

    # 20. Batch time trend
    print("  - Batch time trend")
    for map_name in all_maps:
        fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT))
        has_data = False
        for m in all_metrics:
            parsed = parse_config_name(m['config'].rsplit('_run', 1)[0])
            if parsed['map'] != map_name:
                continue
            if m['batch_times']:
                ax.plot(range(len(m['batch_times'])), m['batch_times'],
                        label=f"bs={parsed['batch_size']}", alpha=0.7, linewidth=1)
                has_data = True

        if has_data:
            ax.set_xlabel('Batch Number', fontsize=FONT_SIZE_LABEL)
            ax.set_ylabel('Batch Compute Time (ms)', fontsize=FONT_SIZE_LABEL)
            ax.tick_params(axis='both', labelsize=FONT_SIZE_TICK_LABELS)
            ax.legend(fontsize=min(LEGEND_SIZE, 16), loc='best')
            ax.grid(True)
            plt.tight_layout(pad=0)
            plt.savefig(PLOTS_DIR / f'batch_time_trend_{map_name}.pdf',
                        dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
        plt.close()

    # Combined batch time trend
    fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT))
    for m in all_metrics:
        if m['batch_times']:
            parsed = parse_config_name(m['config'].rsplit('_run', 1)[0])
            ax.plot(range(len(m['batch_times'])), m['batch_times'],
                    label=f"bs={parsed['batch_size']},{parsed['map']}", alpha=0.5, linewidth=1)

    ax.set_xlabel('Batch Number', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Batch Compute Time (ms)', fontsize=FONT_SIZE_LABEL)
    ax.tick_params(axis='both', labelsize=FONT_SIZE_TICK_LABELS)
    ax.grid(True)
    plt.tight_layout(pad=0)
    plt.savefig(PLOTS_DIR / 'batch_time_trend.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()

    # ==================== RRT*-Specific Metrics ====================

    # 10. Convergence curve
    print("  - Convergence curves")
    for map_name in all_maps:
        fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT))
        has_data = False
        for m in all_metrics:
            parsed = parse_config_name(m['config'].rsplit('_run', 1)[0])
            if parsed['map'] != map_name:
                continue
            if m['convergence_data']:
                iters = [d[0] for d in m['convergence_data']]
                costs = [d[1] for d in m['convergence_data']]
                # Filter out infinity values
                valid = [(i, c) for i, c in zip(iters, costs) if c < 1e10]
                if valid:
                    vi, vc = zip(*valid)
                    ax.plot(vi, vc,
                            label=f"bs={parsed['batch_size']},{parsed['mode']}",
                            alpha=0.7, linewidth=1.5)
                    has_data = True

        if has_data:
            ax.set_xlabel('Iteration', fontsize=FONT_SIZE_LABEL)
            ax.set_ylabel('Best Path Cost', fontsize=FONT_SIZE_LABEL)
            ax.tick_params(axis='both', labelsize=FONT_SIZE_TICK_LABELS)
            ax.legend(fontsize=min(LEGEND_SIZE, 14), loc='best')
            ax.grid(True)
            plt.tight_layout(pad=0)
            plt.savefig(PLOTS_DIR / f'convergence_curve_{map_name}.pdf',
                        dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
        plt.close()

        # Save convergence data as CSV
        conv_rows = []
        for m in all_metrics:
            parsed = parse_config_name(m['config'].rsplit('_run', 1)[0])
            if parsed['map'] != map_name:
                continue
            for d in m['convergence_data']:
                conv_rows.append({
                    'config': m['config'],
                    'iteration': d[0],
                    'best_cost': d[1],
                    'tree_size': d[2],
                })
        if conv_rows:
            conv_df = pd.DataFrame(conv_rows)
            conv_df.to_csv(CONVERGENCE_DIR / f'{map_name}_convergence.csv', index=False)

    # 11. Convergence by map comparison
    print("  - Convergence by map")
    if len(all_maps) > 1:
        fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT))
        colors = {'depot': 'C0', 'warehouse': 'C1'}
        for m in all_metrics:
            parsed = parse_config_name(m['config'].rsplit('_run', 1)[0])
            if m['convergence_data']:
                valid = [(d[0], d[1]) for d in m['convergence_data'] if d[1] < 1e10]
                if valid:
                    vi, vc = zip(*valid)
                    ax.plot(vi, vc, color=colors.get(parsed['map'], 'C2'),
                            alpha=0.4, linewidth=1)

        # Legend by map
        legend_elements = [Patch(facecolor=colors.get(m, 'C2'), label=m) for m in all_maps]
        ax.legend(handles=legend_elements, fontsize=LEGEND_SIZE)
        ax.set_xlabel('Iteration', fontsize=FONT_SIZE_LABEL)
        ax.set_ylabel('Best Path Cost', fontsize=FONT_SIZE_LABEL)
        ax.tick_params(axis='both', labelsize=FONT_SIZE_TICK_LABELS)
        ax.grid(True)
        plt.tight_layout(pad=0)
        plt.savefig(PLOTS_DIR / 'convergence_by_map.pdf',
                    dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
        plt.close()

    # 12. First solution iteration (from RRT* result events or convergence data)
    print("  - First solution iteration")
    # Estimate first solution iteration from convergence data
    first_sol_data = []
    for m in all_metrics:
        parsed = parse_config_name(m['config'].rsplit('_run', 1)[0])
        if m['convergence_data']:
            finite_costs = [d for d in m['convergence_data'] if d[1] < 1e10]
            if finite_costs:
                first_sol_data.append({
                    **parsed,
                    'first_solution_iteration': finite_costs[0][0],
                })

    if first_sol_data:
        fsi_df = pd.DataFrame(first_sol_data)
        # Average by config
        fsi_agg = fsi_df.groupby(['batch_size', 'mode', 'threading', 'map']).agg(
            avg_first_sol=('first_solution_iteration', 'mean')
        ).reset_index()

        for map_name in all_maps:
            map_data = fsi_agg[fsi_agg['map'] == map_name]
            if map_data.empty:
                continue
            fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT))
            all_batch_sizes = sorted(map_data['batch_size'].unique())
            x = np.arange(len(all_batch_sizes))
            width = 0.2

            for i, mode in enumerate(['reactive', 'proactive']):
                for j, threading in enumerate(['single', 'multi']):
                    data = map_data[(map_data['mode'] == mode) & (map_data['threading'] == threading)]
                    if data.empty:
                        continue
                    y_values = []
                    x_positions = []
                    for bs in all_batch_sizes:
                        bd = data[data['batch_size'] == bs]
                        if not bd.empty:
                            y_values.append(bd['avg_first_sol'].iloc[0])
                            x_positions.append(all_batch_sizes.index(bs))
                    if y_values:
                        offset = (i * 2 + j - 1.5) * width
                        ax.bar(np.array(x_positions) + offset, y_values, width,
                               label=f'{mode}-{threading}')

            ax.set_xlabel('Batch Size', fontsize=FONT_SIZE_LABEL)
            ax.set_ylabel('First Solution Iteration', fontsize=FONT_SIZE_LABEL)
            ax.set_xticks(x)
            ax.set_xticklabels(all_batch_sizes, fontsize=FONT_SIZE_TICK_LABELS)
            ax.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
            ax.grid(True, axis='y')
            plt.tight_layout(pad=0)
            plt.savefig(PLOTS_DIR / f'first_solution_iteration_{map_name}.pdf',
                        dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
            plt.close()

    # 13. Best cost vs batch size
    print("  - Best cost vs batch size")
    make_grouped_bar_chart(df, 'avg_final_best_cost',
                           'Best Path Cost at Cancellation',
                           'best_cost_vs_batch_size.pdf')
    for map_name in all_maps:
        make_grouped_bar_chart(df, 'avg_final_best_cost',
                               'Best Path Cost at Cancellation',
                               f'best_cost_vs_batch_size_{map_name}.pdf',
                               map_filter=map_name)

    # 14. Tree size vs iterations (from convergence data)
    print("  - Tree size vs iterations")
    for map_name in all_maps:
        fig, ax = plt.subplots(figsize=(PLOT_WIDTH, PLOT_HEIGHT))
        has_data = False
        for m in all_metrics:
            parsed = parse_config_name(m['config'].rsplit('_run', 1)[0])
            if parsed['map'] != map_name:
                continue
            if m['convergence_data']:
                iters = [d[0] for d in m['convergence_data']]
                sizes = [d[2] for d in m['convergence_data']]
                ax.plot(iters, sizes,
                        label=f"bs={parsed['batch_size']},{parsed['mode']}",
                        alpha=0.7, linewidth=1.5)
                has_data = True

        if has_data:
            ax.set_xlabel('Iteration', fontsize=FONT_SIZE_LABEL)
            ax.set_ylabel('Tree Size (nodes)', fontsize=FONT_SIZE_LABEL)
            ax.tick_params(axis='both', labelsize=FONT_SIZE_TICK_LABELS)
            ax.legend(fontsize=min(LEGEND_SIZE, 14), loc='best')
            ax.grid(True)
            plt.tight_layout(pad=0)
            plt.savefig(PLOTS_DIR / f'tree_size_vs_iterations_{map_name}.pdf',
                        dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
        plt.close()

    # 9. Legend
    print("  - Creating separate legend")
    legend_elements = [
        Patch(facecolor=f'C{i*2+j}', label=f'{mode}-{threading}')
        for i, mode in enumerate(['reactive', 'proactive'])
        for j, threading in enumerate(['single', 'multi'])
    ]
    fig_legend = plt.figure(figsize=(PLOT_WIDTH, 0.5))
    fig_legend.legend(handles=legend_elements, loc='center', ncol=4,
                      fontsize=LEGEND_SIZE, frameon=False)
    plt.savefig(PLOTS_DIR / 'legend.pdf', dpi=PLOT_DPI,
                bbox_inches='tight', pad_inches=0)
    plt.close()

    print(f"  All plots saved to: {PLOTS_DIR}")


def main():
    print("========================================")
    print("RRT* Experiment Evaluation")
    print("========================================")
    print()

    # Find all trace directories
    trace_dirs = sorted([
        d for d in TRACE_DIR.iterdir()
        if d.is_dir() and d.name.startswith('batch_')
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

        print(f"    Batches: {metrics['total_batches']}, "
              f"Iterations: {metrics['total_iterations']}, "
              f"Avg time/batch: {metrics['avg_time_per_batch']:.2f}ms, "
              f"Overhead ratio: {metrics['avg_overhead_ratio']:.2f}%")

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

    aggregated_json = RESULTS_DIR / 'aggregated_results.json'
    with open(aggregated_json, 'w') as f:
        json.dump(aggregated, f, indent=2)
    print(f"  Saved to: {aggregated_json}")

    # Generate plots
    generate_plots(aggregated, all_metrics)

    # Generate map visualizations with start/goal positions
    print("\nGenerating map visualizations...")
    try:
        from visualize_maps import visualize_map, parse_yaml_flat, MAP_CONFIGS, MAPS_DIR
        for map_name, config in MAP_CONFIGS.items():
            yaml_path = MAPS_DIR / f"{map_name}.yaml"
            if yaml_path.exists():
                visualize_map(yaml_path, map_name, config, PLOTS_DIR, fmt='pdf')
                visualize_map(yaml_path, map_name, config, PLOTS_DIR, fmt='png')
            else:
                print(f"  WARNING: {yaml_path} not found, skipping {map_name}")
    except Exception as e:
        print(f"  WARNING: Could not generate map visualizations: {e}")

    # Print summary
    print("\n========================================")
    print("Summary Statistics")
    print("========================================")
    print(f"\nTotal configurations tested: {len(aggregated)}")
    print(f"Total experiment runs: {len(all_metrics)}")

    print("\nBatch Size Performance:")
    for batch_size in sorted(aggregated_df['batch_size'].unique()):
        data = aggregated_df[aggregated_df['batch_size'] == batch_size]
        avg_throughput = (data['batch_size'] / (data['avg_time_per_batch'] / 1000.0)).mean()
        avg_overhead = data['avg_overhead_ratio'].mean()
        print(f"  Batch {batch_size:6d}: {avg_throughput:10.2f} iter/sec, "
              f"overhead ratio: {avg_overhead:.2f}%")

    print("\n========================================")
    print("Evaluation Complete!")
    print("========================================")
    print(f"\nResults directory: {RESULTS_DIR}")
    print(f"Plots directory: {PLOTS_DIR}")
    print(f"Convergence data: {CONVERGENCE_DIR}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
