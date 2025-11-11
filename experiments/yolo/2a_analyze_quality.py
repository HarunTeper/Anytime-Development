#!/usr/bin/env python3
"""
Step 2a: Quality Analysis

Analyzes baseline traces (from Step 1) to determine optimal cancellation points.

Key Questions:
- After how many layers can we cancel while maintaining quality?
- How does detection quality progress across layers?
- What are the recommended cancellation thresholds?

Configuration:
- FILTER_BY_CLASS: Set to True to filter by TARGET_CLASS_ID, False for all detections
- TARGET_CLASS_ID: 9 = traffic light in COCO dataset

Input:  traces/phase1_baseline_trial{1,2,3}/
Output: results/quality_analysis/
"""

import os
import sys
import json
import subprocess
from pathlib import Path
from collections import defaultdict
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

# Configuration
EXPERIMENT_DIR = Path("/home/vscode/workspace/experiments/yolo")
TRACE_DIR = EXPERIMENT_DIR / "traces"
RESULTS_DIR = EXPERIMENT_DIR / "results"
PLOTS_DIR = RESULTS_DIR / "plots"
QUALITY_DIR = RESULTS_DIR / "quality_analysis"

# Filter settings
# Set to True to filter by specific class, False for all detections
FILTER_BY_CLASS = True
# Target class ID to filter for (9 = traffic light in COCO dataset)
TARGET_CLASS_ID = 9

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
QUALITY_DIR.mkdir(exist_ok=True)


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


def analyze_quality_progression(trace_dir):
    """
    Analyze quality progression for each image across layers
    Returns detailed metrics per image
    If FILTER_BY_CLASS is True, filters detections by TARGET_CLASS_ID
    """
    events = parse_trace_directory(trace_dir)

    if not events:
        return None

    # Track per-image quality progression
    images = []
    current_image = {
        'image_id': 0,
        # layer_num -> num_detections (filtered if enabled)
        'layer_detections': {},
        'layer_times': {},  # layer_num -> timestamp
        'layer_computation_times': {},  # layer_num -> computation time (ns)
        # layer_num -> exit calculation time (ns)
        'exit_calculation_times': {},
        'layer_start_times': {},  # layer_num -> start timestamp
        'exit_calc_start_times': {},  # layer_num -> exit calc start timestamp
        'final_detections': 0,
        'max_layer': 0,
    }

    image_counter = 0

    for event in events:
        if event.event_name == 'anytime_base_activate':
            # New goal/image started
            if current_image['layer_detections']:
                images.append(current_image)

            image_counter += 1
            current_image = {
                'image_id': image_counter,
                'layer_detections': {},
                'layer_times': {},
                'layer_computation_times': {},
                'exit_calculation_times': {},
                'layer_start_times': {},
                'exit_calc_start_times': {},
                'final_detections': 0,
                'max_layer': 0,
            }

        elif event.event_name == 'yolo_layer_start':
            layer_num = event.fields.get('layer_num', 0)
            # Store with the start layer number for matching later
            current_image['layer_start_times'][layer_num] = event.timestamp

        elif event.event_name == 'yolo_layer_end':
            layer_num = event.fields.get('layer_num', 0)
            current_image['layer_times'][layer_num] = event.timestamp
            current_image['max_layer'] = max(
                current_image['max_layer'], layer_num)

            # Calculate layer computation time
            # yolo_layer_start uses layer_num-1, yolo_layer_end uses layer_num
            start_layer_num = layer_num - 1
            if start_layer_num in current_image['layer_start_times']:
                start_time = current_image['layer_start_times'][start_layer_num]
                computation_time = event.timestamp - start_time
                current_image['layer_computation_times'][layer_num] = computation_time

        elif event.event_name == 'yolo_exit_calculation_start':
            # Mark that we're starting exit calculation for this layer
            layer_num = event.fields.get('layer_num', 0)
            current_image['current_exit_layer'] = layer_num
            current_image['exit_calc_start_times'][layer_num] = event.timestamp

        elif event.event_name == 'yolo_exit_calculation_end':
            layer_num = event.fields.get('layer_num', 0)
            # Calculate exit calculation time
            if layer_num in current_image['exit_calc_start_times']:
                start_time = current_image['exit_calc_start_times'][layer_num]
                calc_time = event.timestamp - start_time
                current_image['exit_calculation_times'][layer_num] = calc_time

        elif event.event_name == 'yolo_detection':
            # Track individual detections for this specific layer
            # Detections come AFTER yolo_exit_calculation_end and belong to layer_num in the event
            layer_num = event.fields.get('layer_num', 0)
            class_id = event.fields.get('class_id', -1)

            # Check if we haven't recorded this layer yet (first detection for this layer)
            if layer_num not in current_image['layer_detections']:
                current_image['layer_detections'][layer_num] = 0

            if FILTER_BY_CLASS:
                # Only count detections of the target class
                if class_id == TARGET_CLASS_ID:
                    current_image['layer_detections'][layer_num] += 1
            else:
                # Count all detections
                current_image['layer_detections'][layer_num] += 1

        elif event.event_name == 'yolo_result':
            # yolo_result is emitted after every layer, so we keep updating final_detections
            # The last one (highest processed_layers) will be the true final result
            processed_layers = event.fields.get('processed_layers', 0)

            if FILTER_BY_CLASS:
                # The final detection count is from the last processed layer
                current_image['final_detections'] = current_image['layer_detections'].get(
                    processed_layers, 0)
                current_image['max_layer'] = max(
                    current_image['max_layer'], processed_layers)
            else:
                # Use total detection count from the event
                total_detections = event.fields.get('total_detections', 0)
                current_image['final_detections'] = total_detections
                current_image['max_layer'] = max(
                    current_image['max_layer'], processed_layers)

    # Don't forget the last image
    if current_image['layer_detections']:
        images.append(current_image)

    return images


def calculate_quality_metrics(images):
    """
    Calculate quality metrics from image data
    Only includes images that have target detections (final_detections > 0)
    """
    print("\n  Calculating quality metrics...")

    # Filter to only images with detections
    images_with_detections = [
        img for img in images if img['final_detections'] > 0]
    images_without_detections = len(images) - len(images_with_detections)

    print(f"  Images with detections: {len(images_with_detections)}")
    print(f"  Images without detections: {images_without_detections}")

    metrics = {
        'total_images': len(images),
        'images_with_detections': len(images_with_detections),
        'images_without_detections': images_without_detections,
        # layer -> [detection counts]
        'layer_detection_progression': defaultdict(list),
        # layer -> [detection/final ratio]
        'layer_quality_ratio': defaultdict(list),
        # layer -> [computation times in ms]
        'layer_computation_times': defaultdict(list),
        # layer -> [exit calculation times in ms]
        'exit_calculation_times': defaultdict(list),
        # Dictionary of threshold -> list of layers where reached
        'threshold_layers': {
            50: [],
            60: [],
            70: [],
            80: [],
            90: [],
            95: [],
            99: [],
        },
    }

    for img in images_with_detections:
        final_count = img['final_detections']

        # Track progression
        for layer_num in sorted(img['layer_detections'].keys()):
            detections = img['layer_detections'][layer_num]
            metrics['layer_detection_progression'][layer_num].append(
                detections)

            # Quality ratio (how close to final result)
            ratio = detections / final_count if final_count > 0 else 0
            metrics['layer_quality_ratio'][layer_num].append(ratio)

    # Track timing data from all images (including those without detections)
    for img in images:
        # Layer computation times
        for layer_num, comp_time in img['layer_computation_times'].items():
            # Convert from nanoseconds to milliseconds
            metrics['layer_computation_times'][layer_num].append(
                comp_time / 1e6)

        # Exit calculation times
        for layer_num, calc_time in img['exit_calculation_times'].items():
            # Convert from nanoseconds to milliseconds
            metrics['exit_calculation_times'][layer_num].append(
                calc_time / 1e6)

        # Find first layer where we reach each quality threshold
        for threshold_pct in [50, 60, 70, 80, 90, 95, 99]:
            threshold = threshold_pct / 100.0
            for layer_num in sorted(img['layer_detections'].keys()):
                detections = img['layer_detections'][layer_num]
                ratio = detections / final_count if final_count > 0 else 0

                if ratio >= threshold:
                    metrics['threshold_layers'][threshold_pct].append(
                        layer_num)
                    break

    # Calculate statistics
    metrics['avg_detections_per_layer'] = {
        layer: {
            'mean': np.mean(counts),
            'std': np.std(counts),
            'min': np.min(counts),
            'max': np.max(counts),
        }
        for layer, counts in metrics['layer_detection_progression'].items()
    }

    metrics['avg_quality_ratio_per_layer'] = {
        layer: {
            'mean': np.mean(ratios),
            'std': np.std(ratios),
            'min': np.min(ratios),
            'max': np.max(ratios),
        }
        for layer, ratios in metrics['layer_quality_ratio'].items()
    }

    # Calculate timing statistics
    metrics['layer_computation_time_stats'] = {
        layer: {
            'mean': np.mean(times),
            'std': np.std(times),
            'min': np.min(times),
            'max': np.max(times),
            'median': np.median(times),
            'count': len(times),
        }
        for layer, times in metrics['layer_computation_times'].items() if len(times) > 0
    }

    metrics['exit_calculation_time_stats'] = {
        layer: {
            'mean': np.mean(times),
            'std': np.std(times),
            'min': np.min(times),
            'max': np.max(times),
            'median': np.median(times),
            'count': len(times),
        }
        for layer, times in metrics['exit_calculation_times'].items() if len(times) > 0
    }

    return metrics


def determine_cancellation_thresholds(metrics):
    """
    Determine recommended cancellation thresholds for multiple quality levels
    """
    print("\n  Determining cancellation thresholds...")

    # Mean-based thresholds: first layer where mean quality ratio reaches threshold
    mean_thresholds = {}
    for threshold_pct in [50, 60, 70, 80, 90, 95, 99]:
        mean_thresholds[f'{threshold_pct}%'] = None
        threshold = threshold_pct / 100.0

        for layer_num in sorted(metrics['avg_quality_ratio_per_layer'].keys()):
            mean_ratio = metrics['avg_quality_ratio_per_layer'][layer_num]['mean']

            if mean_ratio >= threshold:
                mean_thresholds[f'{threshold_pct}%'] = layer_num
                break

    # Per-image statistics: when each image first reaches each threshold
    threshold_stats = {}
    for threshold_pct in [50, 60, 70, 80, 90, 95, 99]:
        layers = metrics['threshold_layers'][threshold_pct]
        if layers:
            threshold_stats[f'{threshold_pct}%'] = {
                'mean': np.mean(layers),
                'median': np.median(layers),
                'std': np.std(layers),
                'min': np.min(layers),
                'max': np.max(layers),
                'count': len(layers),
            }
        else:
            threshold_stats[f'{threshold_pct}%'] = None

    return mean_thresholds, threshold_stats


def plot_detection_progression(metrics):
    """
    Plot detection count progression across layers
    """
    print("\n  Creating detection progression plot...")

    layers = sorted(metrics['layer_detection_progression'].keys())
    means = [np.mean(metrics['layer_detection_progression'][l])
             for l in layers]
    stds = [np.std(metrics['layer_detection_progression'][l]) for l in layers]

    fig, ax = plt.subplots(figsize=(12, 6))

    ax.errorbar(layers, means, yerr=stds, marker='o',
                capsize=5, linewidth=2, markersize=8)
    ax.set_xlabel('Layer Number', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Average Detection Count', fontsize=FONT_SIZE_LABEL)
    ax.tick_params(axis='both', labelsize=FONT_SIZE_TICK_LABELS)

    if FILTER_BY_CLASS:
        title = f'YOLO Detection Count Progression Across Layers (Class {TARGET_CLASS_ID})'
    else:
        title = 'YOLO Detection Count Progression Across Layers (All Classes)'
    ax.set_title(title, fontsize=FONT_SIZE_TITLE)
    ax.grid(True, alpha=0.3)

    plt.tight_layout(pad=0)
    plt.savefig(QUALITY_DIR / 'detection_progression.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()
    print(f"    Saved: {QUALITY_DIR / 'detection_progression.pdf'}")


def plot_quality_ratio_progression(metrics):
    """
    Plot quality ratio (detections/final) progression across layers
    """
    print("\n  Creating quality ratio progression plot...")

    layers = sorted(metrics['layer_quality_ratio'].keys())
    if not layers:
        print("    No data for quality ratio progression plot")
        return

    means = [np.mean(metrics['layer_quality_ratio'][l]) for l in layers]
    stds = [np.std(metrics['layer_quality_ratio'][l]) for l in layers]

    fig, ax = plt.subplots(figsize=(12, 6))

    ax.errorbar(layers, means, yerr=stds, marker='o',
                capsize=5, linewidth=2, markersize=8)
    ax.axhline(y=0.90, color='g', linestyle='--', label='90% Quality')
    ax.axhline(y=0.95, color='orange', linestyle='--', label='95% Quality')
    ax.axhline(y=0.99, color='r', linestyle='--', label='99% Quality')

    ax.set_xlabel('Layer Number', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Quality Ratio (Detections / Final)',
                  fontsize=FONT_SIZE_LABEL)
    ax.tick_params(axis='both', labelsize=FONT_SIZE_TICK_LABELS)

    if FILTER_BY_CLASS:
        title = f'YOLO Detection Quality Progression Across Layers (Class {TARGET_CLASS_ID})'
    else:
        title = 'YOLO Detection Quality Progression Across Layers (All Classes)'
    ax.set_title(title, fontsize=FONT_SIZE_TITLE)
    ax.set_ylim([0, 1.05])
    ax.legend(fontsize=LEGEND_SIZE)
    ax.grid(True, alpha=0.3)

    plt.tight_layout(pad=0)
    plt.savefig(QUALITY_DIR / 'quality_ratio_progression.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()
    print(f"    Saved: {QUALITY_DIR / 'quality_ratio_progression.pdf'}")


def plot_cancellation_histogram(metrics):
    """
    Plot histogram of when images reach sufficient quality at different thresholds
    """
    print("\n  Creating cancellation histograms...")

    # Create subplots for different thresholds
    thresholds_to_plot = [70, 80, 90, 95, 99]
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    axes = axes.flatten()

    plotted = False
    for idx, threshold_pct in enumerate(thresholds_to_plot):
        ax = axes[idx]
        layers = metrics['threshold_layers'][threshold_pct]

        if not layers:
            ax.text(0.5, 0.5, f'No data for {threshold_pct}%',
                    ha='center', va='center', transform=ax.transAxes)
            ax.set_title(f'{threshold_pct}% Quality Threshold')
            continue

        plotted = True
        ax.hist(layers, bins=20, edgecolor='black', alpha=0.7)
        ax.axvline(np.mean(layers), color='r', linestyle='--',
                   linewidth=2, label=f"Mean: {np.mean(layers):.1f}")
        ax.axvline(np.median(layers), color='g', linestyle='--',
                   linewidth=2, label=f"Median: {np.median(layers):.1f}")

        ax.set_xlabel('Layer Number', fontsize=FONT_SIZE_LABEL)
        ax.set_ylabel('Number of Images', fontsize=FONT_SIZE_LABEL)
        ax.set_title(f'{threshold_pct}% Quality Threshold',
                     fontsize=FONT_SIZE_TITLE)
        ax.tick_params(axis='both', labelsize=FONT_SIZE_TICK_LABELS)
        ax.legend(fontsize=LEGEND_SIZE)
        ax.grid(True, alpha=0.3, axis='y')

    # Remove extra subplot
    fig.delaxes(axes[5])

    if FILTER_BY_CLASS:
        fig.suptitle(f'Distribution: After How Many Layers Can We Cancel? (Class {TARGET_CLASS_ID})',
                     fontsize=14, y=0.995)
    else:
        fig.suptitle('Distribution: After How Many Layers Can We Cancel? (All Classes)',
                     fontsize=14, y=0.995)

    plt.tight_layout(pad=0)

    if plotted:
        plt.savefig(QUALITY_DIR / 'cancellation_histograms.pdf',
                    dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
        print(f"    Saved: {QUALITY_DIR / 'cancellation_histograms.pdf'}")
    else:
        print("    No data for histograms")

    plt.close()


def plot_layer_wise_boxplot(metrics):
    """
    Create box plot showing quality distribution at each layer
    """
    print("\n  Creating layer-wise quality boxplot...")

    layers = sorted(metrics['layer_quality_ratio'].keys())
    data = [metrics['layer_quality_ratio'][l] for l in layers]

    if not layers or not data or all(len(d) == 0 for d in data):
        print("    No data for boxplot")
        return

    fig, ax = plt.subplots(figsize=(14, 6))

    bp = ax.boxplot(data, tick_labels=layers, patch_artist=True)

    # Color boxes
    for patch in bp['boxes']:
        patch.set_facecolor('lightblue')

    # Add threshold lines
    ax.axhline(y=0.90, color='g', linestyle='--',
               alpha=0.5, label='90% Quality')
    ax.axhline(y=0.95, color='orange', linestyle='--',
               alpha=0.5, label='95% Quality')
    ax.axhline(y=0.99, color='r', linestyle='--',
               alpha=0.5, label='99% Quality')

    ax.set_xlabel('Layer Number', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Quality Ratio', fontsize=FONT_SIZE_LABEL)
    ax.tick_params(axis='both', labelsize=FONT_SIZE_TICK_LABELS)

    if FILTER_BY_CLASS:
        title = f'Quality Distribution at Each Layer (Class {TARGET_CLASS_ID})'
    else:
        title = 'Quality Distribution at Each Layer (All Classes)'
    ax.set_title(title, fontsize=FONT_SIZE_TITLE)
    ax.set_ylim([0, 1.05])
    ax.legend(fontsize=LEGEND_SIZE)
    ax.grid(True, alpha=0.3, axis='y')

    plt.tight_layout(pad=0)
    plt.savefig(QUALITY_DIR / 'quality_boxplot.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()
    print(f"    Saved: {QUALITY_DIR / 'quality_boxplot.pdf'}")


def plot_layer_computation_times(metrics):
    """
    Plot layer computation times across layers
    """
    print("\n  Creating layer computation time plot...")

    if not metrics['layer_computation_time_stats']:
        print("    No data for layer computation times")
        return

    layers = sorted(metrics['layer_computation_time_stats'].keys())
    means = [metrics['layer_computation_time_stats'][l]['mean']
             for l in layers]
    stds = [metrics['layer_computation_time_stats'][l]['std'] for l in layers]

    fig, ax = plt.subplots(figsize=(12, 6))

    ax.errorbar(layers, means, yerr=stds, marker='o',
                capsize=5, linewidth=2, markersize=8, color='blue')
    ax.set_xlabel('Layer Number', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Computation Time (ms)', fontsize=FONT_SIZE_LABEL)
    ax.set_title('Layer Computation Time Across Layers',
                 fontsize=FONT_SIZE_TITLE)
    ax.tick_params(axis='both', labelsize=FONT_SIZE_TICK_LABELS)
    ax.grid(True, alpha=0.3)

    plt.tight_layout(pad=0)
    plt.savefig(QUALITY_DIR / 'layer_computation_times.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()
    print(f"    Saved: {QUALITY_DIR / 'layer_computation_times.pdf'}")


def plot_exit_calculation_times(metrics):
    """
    Plot exit/result calculation times across layers
    """
    print("\n  Creating exit calculation time plot...")

    if not metrics['exit_calculation_time_stats']:
        print("    No data for exit calculation times")
        return

    layers = sorted(metrics['exit_calculation_time_stats'].keys())
    means = [metrics['exit_calculation_time_stats'][l]['mean'] for l in layers]
    stds = [metrics['exit_calculation_time_stats'][l]['std'] for l in layers]

    fig, ax = plt.subplots(figsize=(12, 6))

    ax.errorbar(layers, means, yerr=stds, marker='s',
                capsize=5, linewidth=2, markersize=8, color='green')
    ax.set_xlabel('Layer Number', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel('Calculation Time (ms)', fontsize=FONT_SIZE_LABEL)
    ax.set_title('Exit/Result Calculation Time Across Layers',
                 fontsize=FONT_SIZE_TITLE)
    ax.tick_params(axis='both', labelsize=FONT_SIZE_TICK_LABELS)
    ax.grid(True, alpha=0.3)

    plt.tight_layout(pad=0)
    plt.savefig(QUALITY_DIR / 'exit_calculation_times.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()
    print(f"    Saved: {QUALITY_DIR / 'exit_calculation_times.pdf'}")


def plot_combined_timing(metrics):
    """
    Plot combined view of layer computation and exit calculation times
    """
    print("\n  Creating combined timing plot...")

    if not metrics['layer_computation_time_stats'] or not metrics['exit_calculation_time_stats']:
        print("    No data for combined timing plot")
        return

    layers_comp = sorted(metrics['layer_computation_time_stats'].keys())
    layers_exit = sorted(metrics['exit_calculation_time_stats'].keys())

    # Use the intersection of layers for comparison
    common_layers = sorted(set(layers_comp) & set(layers_exit))

    if not common_layers:
        print("    No common layers for combined plot")
        return

    comp_means = [metrics['layer_computation_time_stats'][l]['mean']
                  for l in common_layers]
    exit_means = [metrics['exit_calculation_time_stats'][l]['mean']
                  for l in common_layers]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))

    # Plot 1: Side-by-side comparison
    x = np.arange(len(common_layers))
    width = 0.35

    ax1.bar(x - width/2, comp_means, width,
            label='Layer Computation', alpha=0.8, color='blue')
    ax1.bar(x + width/2, exit_means, width,
            label='Exit Calculation', alpha=0.8, color='green')
    ax1.set_xlabel('Layer Number', fontsize=FONT_SIZE_LABEL)
    ax1.set_ylabel('Time (ms)', fontsize=FONT_SIZE_LABEL)
    ax1.set_title('Layer Computation vs Exit Calculation Time',
                  fontsize=FONT_SIZE_TITLE)
    ax1.set_xticks(x)
    ax1.set_xticklabels(common_layers, fontsize=FONT_SIZE_TICK_LABELS)
    ax1.tick_params(axis='y', labelsize=FONT_SIZE_TICK_LABELS)
    ax1.legend(fontsize=LEGEND_SIZE)
    ax1.grid(True, alpha=0.3, axis='y')

    # Plot 2: Stacked view showing total time per layer
    total_means = [comp_means[i] + exit_means[i]
                   for i in range(len(common_layers))]

    ax2.bar(common_layers, comp_means,
            label='Layer Computation', alpha=0.8, color='blue')
    ax2.bar(common_layers, exit_means, bottom=comp_means,
            label='Exit Calculation', alpha=0.8, color='green')
    ax2.set_xlabel('Layer Number', fontsize=FONT_SIZE_LABEL)
    ax2.set_ylabel('Time (ms)', fontsize=FONT_SIZE_LABEL)
    ax2.set_title('Total Time per Layer (Stacked)', fontsize=FONT_SIZE_TITLE)
    ax2.tick_params(axis='both', labelsize=FONT_SIZE_TICK_LABELS)
    ax2.legend(fontsize=LEGEND_SIZE)
    ax2.grid(True, alpha=0.3, axis='y')

    plt.tight_layout(pad=0)
    plt.savefig(QUALITY_DIR / 'combined_timing.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()
    print(f"    Saved: {QUALITY_DIR / 'combined_timing.pdf'}")


def plot_cumulative_timing(metrics):
    """
    Plot cumulative timing showing total runtime up to each layer
    This shows the total cost if we cancel at each specific layer
    """
    print("\n  Creating cumulative timing plot...")

    if not metrics['layer_computation_time_stats'] or not metrics['exit_calculation_time_stats']:
        print("    No data for cumulative timing plot")
        return

    layers_comp = sorted(metrics['layer_computation_time_stats'].keys())
    layers_exit = sorted(metrics['exit_calculation_time_stats'].keys())

    # Use the intersection of layers for comparison
    common_layers = sorted(set(layers_comp) & set(layers_exit))

    if not common_layers:
        print("    No common layers for cumulative plot")
        return

    comp_means = [metrics['layer_computation_time_stats'][l]['mean']
                  for l in common_layers]
    exit_means = [metrics['exit_calculation_time_stats'][l]['mean']
                  for l in common_layers]

    # Calculate cumulative sums
    cumulative_comp = np.cumsum(comp_means)
    cumulative_exit = np.cumsum(exit_means)
    cumulative_total = cumulative_comp + cumulative_exit

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))

    # Plot 1: Separate cumulative times for computation and exit calculation
    ax1.plot(common_layers, cumulative_comp, marker='o',
             linewidth=2, markersize=6, label='Cumulative Layer Computation', color='blue')
    ax1.plot(common_layers, cumulative_exit, marker='s',
             linewidth=2, markersize=6, label='Cumulative Exit Calculation', color='green')
    ax1.plot(common_layers, cumulative_total, marker='^',
             linewidth=2, markersize=6, label='Total Cumulative Time', color='red', linestyle='--')
    ax1.set_xlabel('Layer Number', fontsize=FONT_SIZE_LABEL)
    ax1.set_ylabel('Cumulative Time (ms)', fontsize=FONT_SIZE_LABEL)
    ax1.set_title('Cumulative Processing Time Up To Each Layer',
                  fontsize=FONT_SIZE_TITLE)
    ax1.tick_params(axis='both', labelsize=FONT_SIZE_TICK_LABELS)
    ax1.legend(fontsize=LEGEND_SIZE)
    ax1.grid(True, alpha=0.3)

    # Plot 2: Stacked area showing cumulative contribution
    ax2.fill_between(common_layers, 0, cumulative_comp,
                     alpha=0.7, label='Layer Computation', color='blue')
    ax2.fill_between(common_layers, cumulative_comp, cumulative_total,
                     alpha=0.7, label='Exit Calculation', color='green')
    ax2.plot(common_layers, cumulative_total, marker='o',
             linewidth=2, markersize=6, color='red', label='Total Runtime')
    ax2.set_xlabel('Layer Number', fontsize=FONT_SIZE_LABEL)
    ax2.set_ylabel('Cumulative Time (ms)', fontsize=FONT_SIZE_LABEL)
    ax2.set_title(
        'Total Runtime if Cancelled at Each Layer (Stacked Area)', fontsize=FONT_SIZE_TITLE)
    ax2.tick_params(axis='both', labelsize=FONT_SIZE_TICK_LABELS)
    ax2.legend(fontsize=LEGEND_SIZE)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout(pad=0)
    plt.savefig(QUALITY_DIR / 'cumulative_timing.pdf',
                dpi=PLOT_DPI, bbox_inches='tight', pad_inches=0)
    plt.close()
    print(f"    Saved: {QUALITY_DIR / 'cumulative_timing.pdf'}")


def export_quality_results(metrics, mean_thresholds, threshold_stats):
    """
    Export quality analysis results to JSON and text
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

    # Create summary report
    report = {
        'analysis_date': datetime.now().isoformat(),
        'filter_by_class': FILTER_BY_CLASS,
        'target_class_id': TARGET_CLASS_ID if FILTER_BY_CLASS else None,
        'total_images_analyzed': metrics['total_images'],
        'images_with_detections': metrics['images_with_detections'],
        'images_without_detections': metrics['images_without_detections'],
        'mean_based_cancellation_thresholds': convert_numpy(mean_thresholds),
        'per_image_threshold_statistics': convert_numpy(threshold_stats),
        'layer_quality_progression': {
            int(k): convert_numpy(v) for k, v in metrics['avg_quality_ratio_per_layer'].items()
        },
        'layer_detection_counts': {
            int(k): convert_numpy(v) for k, v in metrics['avg_detections_per_layer'].items()
        },
        'layer_computation_time_stats': {
            int(k): convert_numpy(v) for k, v in metrics['layer_computation_time_stats'].items()
        },
        'exit_calculation_time_stats': {
            int(k): convert_numpy(v) for k, v in metrics['exit_calculation_time_stats'].items()
        },
    }

    # Save JSON
    json_path = QUALITY_DIR / 'quality_analysis.json'
    with open(json_path, 'w') as f:
        json.dump(report, f, indent=2)
    print(f"    Saved: {json_path}")

    # Create human-readable summary
    summary_path = QUALITY_DIR / 'quality_summary.txt'
    with open(summary_path, 'w') as f:
        f.write("="*80 + "\n")
        f.write("YOLO QUALITY ANALYSIS SUMMARY\n")
        f.write("="*80 + "\n\n")

        f.write(
            f"Analysis Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")

        if FILTER_BY_CLASS:
            f.write(f"Filter Mode: Class ID {TARGET_CLASS_ID}\n")
        else:
            f.write(f"Filter Mode: All Classes\n")

        f.write(f"Total Images Analyzed: {metrics['total_images']}\n")
        f.write(
            f"Images with Detections: {metrics['images_with_detections']}\n")
        f.write(
            f"Images without Detections: {metrics['images_without_detections']}\n\n")

        f.write("MEAN-BASED CANCELLATION THRESHOLDS:\n")
        f.write("(First layer where average quality reaches threshold)\n")
        f.write("-" * 80 + "\n")
        for quality, layer in mean_thresholds.items():
            if layer:
                f.write(f"  {quality} Quality: Cancel after layer {layer}\n")
            else:
                f.write(f"  {quality} Quality: Never reached\n")
        f.write("\n")

        f.write("PER-IMAGE THRESHOLD STATISTICS:\n")
        f.write("(Statistics on when individual images reach each threshold)\n")
        f.write("-" * 80 + "\n")
        for threshold_pct in [50, 60, 70, 80, 90, 95, 99]:
            threshold_key = f'{threshold_pct}%'
            stats = threshold_stats.get(threshold_key)
            if stats:
                f.write(
                    f"\n  {threshold_key} Quality ({stats['count']} images):\n")
                f.write(f"    Mean:   {stats['mean']:.2f} layers\n")
                f.write(f"    Median: {stats['median']:.2f} layers\n")
                f.write(f"    Std:    {stats['std']:.2f} layers\n")
                f.write(f"    Min:    {stats['min']} layers\n")
                f.write(f"    Max:    {stats['max']} layers\n")
            else:
                f.write(
                    f"\n  {threshold_key} Quality: No images reached this threshold\n")
        f.write("\n")

        f.write("QUALITY PROGRESSION BY LAYER:\n")
        f.write("-" * 80 + "\n")
        for layer in sorted(metrics['avg_quality_ratio_per_layer'].keys()):
            stats = metrics['avg_quality_ratio_per_layer'][layer]
            f.write(
                f"  Layer {layer:2d}: {stats['mean']:.3f} ± {stats['std']:.3f} ")
            f.write(f"(min: {stats['min']:.3f}, max: {stats['max']:.3f})\n")

        f.write("\n")
        f.write("LAYER COMPUTATION TIMES:\n")
        f.write("-" * 80 + "\n")
        if metrics['layer_computation_time_stats']:
            for layer in sorted(metrics['layer_computation_time_stats'].keys()):
                stats = metrics['layer_computation_time_stats'][layer]
                f.write(
                    f"  Layer {layer:2d}: {stats['mean']:.3f} ± {stats['std']:.3f} ms ")
                f.write(f"(median: {stats['median']:.3f} ms, ")
                f.write(
                    f"min: {stats['min']:.3f} ms, max: {stats['max']:.3f} ms, ")
                f.write(f"n={stats['count']})\n")
        else:
            f.write("  No computation time data available\n")

        f.write("\n")
        f.write("EXIT/RESULT CALCULATION TIMES:\n")
        f.write("-" * 80 + "\n")
        if metrics['exit_calculation_time_stats']:
            for layer in sorted(metrics['exit_calculation_time_stats'].keys()):
                stats = metrics['exit_calculation_time_stats'][layer]
                f.write(
                    f"  Layer {layer:2d}: {stats['mean']:.3f} ± {stats['std']:.3f} ms ")
                f.write(f"(median: {stats['median']:.3f} ms, ")
                f.write(
                    f"min: {stats['min']:.3f} ms, max: {stats['max']:.3f} ms, ")
                f.write(f"n={stats['count']})\n")
        else:
            f.write("  No exit calculation time data available\n")

        f.write("\n")
        f.write("TOTAL TIME PER LAYER (Computation + Exit Calculation):\n")
        f.write("-" * 80 + "\n")
        if metrics['layer_computation_time_stats'] and metrics['exit_calculation_time_stats']:
            common_layers = sorted(set(metrics['layer_computation_time_stats'].keys()) &
                                   set(metrics['exit_calculation_time_stats'].keys()))
            for layer in common_layers:
                comp_mean = metrics['layer_computation_time_stats'][layer]['mean']
                exit_mean = metrics['exit_calculation_time_stats'][layer]['mean']
                total_mean = comp_mean + exit_mean
                f.write(f"  Layer {layer:2d}: {total_mean:.3f} ms ")
                f.write(
                    f"(computation: {comp_mean:.3f} ms, exit calc: {exit_mean:.3f} ms)\n")
        else:
            f.write("  Insufficient timing data for total time calculation\n")

        f.write("\n" + "="*80 + "\n")

    print(f"    Saved: {summary_path}")

    # Print to console
    with open(summary_path, 'r') as f:
        print("\n" + f.read())


def main():
    """Main quality analysis function"""
    print("="*80)
    print("STEP 2a: QUALITY ANALYSIS")
    print("="*80)
    print(f"Input:  {TRACE_DIR}")
    print(f"Output: {QUALITY_DIR}")

    if FILTER_BY_CLASS:
        print(f"Filter: Class ID {TARGET_CLASS_ID} only")
    else:
        print(f"Filter: All detections")

    # Find Phase 1 baseline traces
    phase1_traces = [d for d in TRACE_DIR.iterdir()
                     if d.is_dir() and ('phase1_baseline' in d.name or 'baseline' in d.name)]

    if not phase1_traces:
        print("\n❌ Error: No Phase 1 baseline traces found!")
        print(f"Please run Step 1 first: ./1_collect_baseline.sh")
        return 1

    print(f"\nFound {len(phase1_traces)} Phase 1 trace directories")

    # Analyze all Phase 1 traces
    print("\nAnalyzing quality progression...")
    all_images = []

    for trace_dir in phase1_traces:
        print(f"\n  Analyzing: {trace_dir.name}")
        images = analyze_quality_progression(trace_dir)
        if images:
            all_images.extend(images)
            print(f"    Found {len(images)} images")

    if not all_images:
        print("\nError: No image data extracted from traces!")
        return 1

    print(f"\nTotal images analyzed: {len(all_images)}")

    # Calculate quality metrics
    metrics = calculate_quality_metrics(all_images)

    # Determine cancellation thresholds
    mean_thresholds, threshold_stats = determine_cancellation_thresholds(
        metrics)

    # Generate plots
    print("\nGenerating plots...")
    plot_detection_progression(metrics)
    plot_quality_ratio_progression(metrics)
    plot_cancellation_histogram(metrics)
    plot_layer_wise_boxplot(metrics)

    # Generate timing plots
    print("\nGenerating timing plots...")
    plot_layer_computation_times(metrics)
    plot_exit_calculation_times(metrics)
    plot_combined_timing(metrics)
    plot_cumulative_timing(metrics)

    # Export results
    export_quality_results(metrics, mean_thresholds, threshold_stats)

    print("\n" + "="*80)
    print("✅ STEP 2a COMPLETE: QUALITY ANALYSIS")
    print("="*80)
    print(f"Results: {QUALITY_DIR}")
    print(f"\nNext step:")
    print(f"  2b. Analyze blocks: python3 2b_analyze_blocks.py")

    return 0


if __name__ == '__main__':
    sys.exit(main())
