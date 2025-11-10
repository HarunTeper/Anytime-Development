#!/usr/bin/env python3
"""
YOLO Block Size Analysis Script

This script analyzes Phase 1 baseline traces to determine total delay for different block sizes.
For each block size (0-25 layers), it calculates the total delay by summing:
- Layer computation times within each block
- Exit calculation time at the end of each block

The script processes all layers (up to layer 25) and shows the cumulative delay
for all blocks needed to complete processing.
"""

import sys
import subprocess
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
PLOTS_DIR = RESULTS_DIR / "plots"
BLOCK_DIR = RESULTS_DIR / "block_analysis"

# Plot configuration
PLOT_WIDTH = 14
PLOT_HEIGHT = 7
PLOT_DPI = 300
FONT_SIZE_TITLE = 20
FONT_SIZE_LABEL = 20
FONT_SIZE_LEGEND = 20
FONT_SIZE_TICK_LABELS = 18
LEGEND_SIZE = 20
MARKER_SIZE = 8
CAPSIZE = 5
LINE_WIDTH = 2

# Create output directories
RESULTS_DIR.mkdir(exist_ok=True)
PLOTS_DIR.mkdir(exist_ok=True)
BLOCK_DIR.mkdir(exist_ok=True)

MAX_LAYER = 25  # Maximum layer number (0-indexed, so 0-24 = 25 layers total)


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


def extract_timing_data(trace_dir):
    """
    Extract layer computation times and exit calculation times from trace
    Returns dict with per-image timing data
    """
    events = parse_trace_directory(trace_dir)

    if not events:
        return None

    # Track per-image timing data
    images = []
    current_image = {
        'image_id': 0,
        'layer_computation_times': {},  # layer_num -> computation time (ns)
        # layer_num -> exit calculation time (ns)
        'exit_calculation_times': {},
        'layer_start_times': {},        # layer_num -> start timestamp
        'exit_calc_start_times': {},    # layer_num -> exit calc start timestamp
    }

    image_counter = 0

    for event in events:
        if event.event_name == 'anytime_base_activate':
            # New goal/image started
            if current_image['layer_computation_times']:
                images.append(current_image)

            image_counter += 1
            current_image = {
                'image_id': image_counter,
                'layer_computation_times': {},
                'exit_calculation_times': {},
                'layer_start_times': {},
                'exit_calc_start_times': {},
            }

        elif event.event_name == 'yolo_layer_start':
            layer_num = event.fields.get('layer_num', 0)
            current_image['layer_start_times'][layer_num] = event.timestamp

        elif event.event_name == 'yolo_layer_end':
            layer_num = event.fields.get('layer_num', 0)

            # Calculate layer computation time
            # yolo_layer_start uses layer_num-1, yolo_layer_end uses layer_num
            start_layer_num = layer_num - 1
            if start_layer_num in current_image['layer_start_times']:
                comp_time = event.timestamp - \
                    current_image['layer_start_times'][start_layer_num]
                current_image['layer_computation_times'][layer_num] = comp_time

        elif event.event_name == 'yolo_exit_calculation_start':
            layer_num = event.fields.get('layer_num', 0)
            current_image['exit_calc_start_times'][layer_num] = event.timestamp

        elif event.event_name == 'yolo_exit_calculation_end':
            layer_num = event.fields.get('layer_num', 0)
            # Calculate exit calculation time
            if layer_num in current_image['exit_calc_start_times']:
                calc_time = event.timestamp - \
                    current_image['exit_calc_start_times'][layer_num]
                current_image['exit_calculation_times'][layer_num] = calc_time

    # Don't forget the last image
    if current_image['layer_computation_times']:
        images.append(current_image)

    return images


def calculate_block_delays(images, block_size):
    """
    Calculate total delay for a given block size across all images.

    A block consists of:
    - Layer computation for layers in the block
    - Exit calculation at the end of the block

    Returns list of per-image block delay data:
    [{
        'image_id': int,
        'blocks': [
            {'block_num': int, 'layers': [int], 'total_delay': float},
            ...
        ],
        'total_delay': float
    }]
    """
    all_image_data = []

    for img in images:
        image_data = {
            'image_id': img['image_id'],
            'blocks': [],
            'total_delay': 0.0
        }

        if block_size == 0:
            # Special case: block_size 0 means no processing (0 delay)
            all_image_data.append(image_data)
            continue

        # Determine which layers we have data for (up to MAX_LAYER)
        available_layers = sorted(
            [l for l in img['layer_computation_times'].keys() if l <= MAX_LAYER])

        if not available_layers:
            all_image_data.append(image_data)
            continue

        # Group layers into blocks
        block_num = 0
        layer_idx = 0

        while layer_idx < len(available_layers):
            block_start_idx = layer_idx
            block_end_idx = min(layer_idx + block_size, len(available_layers))

            block_layers = available_layers[block_start_idx:block_end_idx]
            last_layer = block_layers[-1]

            # Sum up layer computation times in this block
            block_comp_time = sum(
                img['layer_computation_times'].get(l, 0) for l in block_layers)

            # Add exit calculation time for the last layer in the block
            block_exit_time = img['exit_calculation_times'].get(last_layer, 0)

            block_total_delay = block_comp_time + block_exit_time

            image_data['blocks'].append({
                'block_num': block_num,
                'layers': block_layers,
                'total_delay': block_total_delay
            })

            image_data['total_delay'] += block_total_delay

            block_num += 1
            layer_idx = block_end_idx

        all_image_data.append(image_data)

    return all_image_data


def analyze_all_block_sizes(all_images):
    """
    Analyze all block sizes from 0 to MAX_LAYER
    Returns statistics for each block size
    """
    print("\n  Analyzing all block sizes...")

    block_size_stats = {}

    for block_size in range(0, MAX_LAYER + 1):
        print(f"    Block size {block_size}...", end='', flush=True)

        # Calculate delays for this block size across all images
        all_image_data = calculate_block_delays(all_images, block_size)

        # Collect statistics
        total_delays = [img['total_delay']
                        for img in all_image_data if img['total_delay'] > 0]
        num_blocks = [len(img['blocks'])
                      for img in all_image_data if img['blocks']]

        if total_delays:
            stats = {
                # Convert ns to ms
                'mean_total_delay_ms': np.mean(total_delays) / 1e6,
                'std_total_delay_ms': np.std(total_delays) / 1e6,
                'min_total_delay_ms': np.min(total_delays) / 1e6,
                'max_total_delay_ms': np.max(total_delays) / 1e6,
                'median_total_delay_ms': np.median(total_delays) / 1e6,
                'mean_num_blocks': np.mean(num_blocks) if num_blocks else 0,
                'all_image_data': all_image_data,
            }
        else:
            stats = {
                'mean_total_delay_ms': 0,
                'std_total_delay_ms': 0,
                'min_total_delay_ms': 0,
                'max_total_delay_ms': 0,
                'median_total_delay_ms': 0,
                'mean_num_blocks': 0,
                'all_image_data': all_image_data,
            }

        block_size_stats[block_size] = stats
        print(f" Done (mean delay: {stats['mean_total_delay_ms']:.2f} ms)")

    return block_size_stats


def plot_block_size_delays(block_size_stats):
    """
    Plot total delay vs block size
    """
    print("\n  Creating block size delay plot...")

    block_sizes = sorted(block_size_stats.keys())
    mean_delays = [block_size_stats[bs]['mean_total_delay_ms']
                   for bs in block_sizes]
    std_delays = [block_size_stats[bs]['std_total_delay_ms']
                  for bs in block_sizes]

    fig, ax = plt.subplots(figsize=(14, 7))

    ax.errorbar(block_sizes, mean_delays, yerr=std_delays, marker='o',
                capsize=5, linewidth=2, markersize=6)
    ax.set_xlabel('Block Size (layers per block)', fontsize=12)
    ax.set_ylabel('Total Delay (ms)', fontsize=12)
    ax.set_title('Total Processing Delay vs Block Size',
                 fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)

    # Highlight block_size = 1 (baseline)
    if 1 in block_sizes:
        baseline_delay = block_size_stats[1]['mean_total_delay_ms']
        ax.axhline(y=baseline_delay, color='r', linestyle='--', alpha=0.5,
                   label=f'Block size 1 baseline: {baseline_delay:.2f} ms')
        ax.legend()

    plt.tight_layout()
    plt.savefig(BLOCK_DIR / 'block_size_delays.png', dpi=300)
    plt.close()
    print(f"    Saved: {BLOCK_DIR / 'block_size_delays.png'}")


def plot_num_blocks_vs_block_size(block_size_stats):
    """
    Plot average number of blocks needed vs block size
    """
    print("\n  Creating number of blocks plot...")

    block_sizes = sorted([bs for bs in block_size_stats.keys() if bs > 0])
    mean_num_blocks = [block_size_stats[bs]['mean_num_blocks']
                       for bs in block_sizes]

    fig, ax = plt.subplots(figsize=(14, 7))

    ax.plot(block_sizes, mean_num_blocks, marker='o',
            linewidth=2, markersize=6, color='green')
    ax.set_xlabel('Block Size (layers per block)', fontsize=12)
    ax.set_ylabel('Average Number of Blocks', fontsize=12)
    ax.set_title('Number of Blocks Needed vs Block Size',
                 fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(BLOCK_DIR / 'num_blocks_vs_block_size.png', dpi=300)
    plt.close()
    print(f"    Saved: {BLOCK_DIR / 'num_blocks_vs_block_size.png'}")


def plot_per_block_delay_distribution(block_size_stats, selected_block_sizes=[1, 3, 5, 8, 16, 25]):
    """
    Plot distribution of per-block delays for selected block sizes
    """
    print("\n  Creating per-block delay distribution plot...")

    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    axes = axes.flatten()

    for idx, block_size in enumerate(selected_block_sizes):
        if idx >= len(axes) or block_size not in block_size_stats:
            continue

        ax = axes[idx]

        # Collect all block delays for this block size
        all_block_delays = []
        for img_data in block_size_stats[block_size]['all_image_data']:
            for block in img_data['blocks']:
                all_block_delays.append(
                    block['total_delay'] / 1e6)  # Convert to ms

        if all_block_delays:
            ax.hist(all_block_delays, bins=30, edgecolor='black',
                    alpha=0.7, color='skyblue')
            ax.axvline(np.mean(all_block_delays), color='r', linestyle='--',
                       linewidth=2, label=f"Mean: {np.mean(all_block_delays):.2f} ms")
            ax.axvline(np.median(all_block_delays), color='g', linestyle='--',
                       linewidth=2, label=f"Median: {np.median(all_block_delays):.2f} ms")
            ax.set_xlabel('Block Delay (ms)', fontsize=10)
            ax.set_ylabel('Frequency', fontsize=10)
            ax.set_title(f'Block Size {block_size}',
                         fontsize=12, fontweight='bold')
            ax.legend(fontsize=9)
            ax.grid(True, alpha=0.3, axis='y')
        else:
            ax.text(0.5, 0.5, 'No data', ha='center',
                    va='center', transform=ax.transAxes)
            ax.set_title(f'Block Size {block_size}',
                         fontsize=12, fontweight='bold')

    fig.suptitle('Per-Block Delay Distribution for Different Block Sizes',
                 fontsize=16, fontweight='bold', y=0.995)
    plt.tight_layout()
    plt.savefig(BLOCK_DIR / 'per_block_delay_distribution.png', dpi=300)
    plt.close()
    print(f"    Saved: {BLOCK_DIR / 'per_block_delay_distribution.png'}")


def plot_detailed_block_breakdown(block_size_stats, selected_block_sizes=[1, 4, 8, 16]):
    """
    Create detailed breakdown showing cumulative delay across blocks for selected block sizes
    """
    print("\n  Creating detailed block breakdown plot...")

    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    axes = axes.flatten()

    for idx, block_size in enumerate(selected_block_sizes):
        if idx >= len(axes) or block_size not in block_size_stats:
            continue

        ax = axes[idx]

        # Get one representative image to show the breakdown
        img_data = block_size_stats[block_size]['all_image_data'][0]

        if not img_data['blocks']:
            ax.text(0.5, 0.5, 'No data', ha='center',
                    va='center', transform=ax.transAxes)
            ax.set_title(f'Block Size {block_size}',
                         fontsize=12, fontweight='bold')
            continue

        block_nums = [b['block_num'] for b in img_data['blocks']]
        block_delays = [b['total_delay'] / 1e6 for b in img_data['blocks']]
        cumulative_delays = np.cumsum(block_delays)

        # Bar plot of individual block delays
        ax2 = ax.twinx()
        ax.bar(block_nums, block_delays, alpha=0.6,
               color='skyblue', label='Block Delay')

        # Line plot of cumulative delay
        ax2.plot(block_nums, cumulative_delays, marker='o', linewidth=2,
                 markersize=6, color='red', label='Cumulative Delay')

        ax.set_xlabel('Block Number', fontsize=10)
        ax.set_ylabel('Block Delay (ms)', fontsize=10, color='skyblue')
        ax2.set_ylabel('Cumulative Delay (ms)', fontsize=10, color='red')
        ax.set_title(f'Block Size {block_size} (Image {img_data["image_id"]})',
                     fontsize=12, fontweight='bold')
        ax.tick_params(axis='y', labelcolor='skyblue')
        ax2.tick_params(axis='y', labelcolor='red')
        ax.grid(True, alpha=0.3, axis='y')

        # Add legends
        lines1, labels1 = ax.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax.legend(lines1 + lines2, labels1 + labels2,
                  loc='upper left', fontsize=9)

    fig.suptitle('Detailed Block Breakdown: Individual and Cumulative Delays',
                 fontsize=16, fontweight='bold', y=0.995)
    plt.tight_layout()
    plt.savefig(BLOCK_DIR / 'detailed_block_breakdown.png', dpi=300)
    plt.close()
    print(f"    Saved: {BLOCK_DIR / 'detailed_block_breakdown.png'}")


def export_block_results(block_size_stats):
    """
    Export block analysis results to JSON and text
    """
    print("\n  Exporting results...")

    # Helper to convert numpy types to native Python types
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

    # Create summary (without full image data to keep JSON manageable)
    summary = {
        'analysis_date': datetime.now().isoformat(),
        'max_layer': MAX_LAYER,
        'block_size_statistics': {}
    }

    for block_size, stats in block_size_stats.items():
        summary['block_size_statistics'][block_size] = {
            'mean_total_delay_ms': convert_numpy(stats['mean_total_delay_ms']),
            'std_total_delay_ms': convert_numpy(stats['std_total_delay_ms']),
            'min_total_delay_ms': convert_numpy(stats['min_total_delay_ms']),
            'max_total_delay_ms': convert_numpy(stats['max_total_delay_ms']),
            'median_total_delay_ms': convert_numpy(stats['median_total_delay_ms']),
            'mean_num_blocks': convert_numpy(stats['mean_num_blocks']),
        }

    # Save JSON
    json_path = BLOCK_DIR / 'block_analysis.json'
    with open(json_path, 'w') as f:
        json.dump(summary, f, indent=2)
    print(f"    Saved: {json_path}")

    # Create human-readable summary
    summary_path = BLOCK_DIR / 'block_summary.txt'
    with open(summary_path, 'w') as f:
        f.write("="*80 + "\n")
        f.write("YOLO BLOCK SIZE ANALYSIS SUMMARY\n")
        f.write("="*80 + "\n\n")

        f.write(
            f"Analysis Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Maximum Layer: {MAX_LAYER}\n\n")

        f.write("BLOCK SIZE STATISTICS:\n")
        f.write("-" * 80 + "\n")
        f.write(
            f"{'Block Size':>12} {'Mean Delay (ms)':>16} {'Std Dev (ms)':>14} {'Median (ms)':>13} {'Avg # Blocks':>14}\n")
        f.write("-" * 80 + "\n")

        for block_size in sorted(block_size_stats.keys()):
            stats = block_size_stats[block_size]
            f.write(f"{block_size:>12} {stats['mean_total_delay_ms']:>16.2f} "
                    f"{stats['std_total_delay_ms']:>14.2f} "
                    f"{stats['median_total_delay_ms']:>13.2f} "
                    f"{stats['mean_num_blocks']:>14.1f}\n")

        f.write("\n" + "="*80 + "\n")
        f.write("KEY INSIGHTS:\n")
        f.write("-" * 80 + "\n")

        # Find optimal block size (minimum mean delay)
        optimal_block_size = min(block_size_stats.keys(),
                                 key=lambda bs: block_size_stats[bs]['mean_total_delay_ms'] if bs > 0 else float('inf'))
        optimal_delay = block_size_stats[optimal_block_size]['mean_total_delay_ms']

        f.write(
            f"\nOptimal block size (minimum mean delay): {optimal_block_size}\n")
        f.write(f"  Mean delay: {optimal_delay:.2f} ms\n")

        # Compare to block_size = 1 (baseline)
        if 1 in block_size_stats:
            baseline_delay = block_size_stats[1]['mean_total_delay_ms']
            improvement = ((baseline_delay - optimal_delay) /
                           baseline_delay) * 100
            f.write(f"\nBaseline (block size 1): {baseline_delay:.2f} ms\n")
            f.write(f"Improvement over baseline: {improvement:.2f}%\n")

        f.write("\n" + "="*80 + "\n")

    print(f"    Saved: {summary_path}")

    # Print to console
    with open(summary_path, 'r') as f:
        print("\n" + f.read())


def main():
    """Main block analysis function"""
    print("="*80)
    print("YOLO BLOCK SIZE ANALYSIS")
    print("="*80)
    print(f"Trace directory: {TRACE_DIR}")
    print(f"Block results: {BLOCK_DIR}")
    print(f"Maximum layer: {MAX_LAYER}")

    # Find Phase 1 baseline traces
    phase1_traces = [d for d in TRACE_DIR.iterdir()
                     if d.is_dir() and ('phase1_baseline' in d.name or 'baseline' in d.name)]

    if not phase1_traces:
        print("\nError: No Phase 1 baseline traces found!")
        print(f"Please run Phase 1 experiments first: ./run_phase1_baseline.sh")
        return 1

    print(f"\nFound {len(phase1_traces)} Phase 1 trace directories")

    # Extract timing data from all traces
    print("\nExtracting timing data from traces...")
    all_images = []

    for trace_dir in phase1_traces:
        print(f"\n  Processing: {trace_dir.name}")
        images = extract_timing_data(trace_dir)
        if images:
            all_images.extend(images)
            print(f"    Extracted data for {len(images)} images")

    if not all_images:
        print("\nError: No timing data extracted from traces!")
        return 1

    print(f"\nTotal images analyzed: {len(all_images)}")

    # Analyze all block sizes
    block_size_stats = analyze_all_block_sizes(all_images)

    # Generate plots
    print("\nGenerating plots...")
    plot_block_size_delays(block_size_stats)
    plot_num_blocks_vs_block_size(block_size_stats)
    plot_per_block_delay_distribution(block_size_stats)
    plot_detailed_block_breakdown(block_size_stats)

    # Export results
    export_block_results(block_size_stats)

    print("\n" + "="*80)
    print("BLOCK ANALYSIS COMPLETE")
    print("="*80)
    print(f"Results saved to: {BLOCK_DIR}")

    return 0


if __name__ == '__main__':
    sys.exit(main())
