#!/usr/bin/env python3
"""
YOLO Quality Analysis Script (Phase 2)

This script analyzes Phase 1 baseline traces to determine optimal cancellation points.
It examines layer-wise detection quality to answer:
- After how many layers can we cancel while maintaining quality?
- How does detection quality progress across layers?
- What are the recommended cancellation thresholds?
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

    # Use babeltrace2 with filter if available (faster)
    try:
        result = subprocess.run(
            ['babeltrace2', '--names', 'none', str(trace_dir)],
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
    anytime_lines = [line for line in result.stdout.split('\n') if 'anytime:' in line]

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
            timestamp = float(timestamp_str.replace(':', '')) * 1e9

            # Extract event name
            event_start = line.find('anytime:')
            if event_start == -1:
                continue
            
            event_name_part = line[event_start:]
            event_name_end = event_name_part.find(':')
            if event_name_end == -1:
                continue
            
            event_name = event_name_part[8:event_name_end]

            # Extract fields (in braces)
            fields_start = line.find('{')
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
    """
    events = parse_trace_directory(trace_dir)
    
    if not events:
        return None
    
    # Track per-image quality progression
    images = []
    current_image = {
        'image_id': 0,
        'layer_detections': {},  # layer_num -> num_detections
        'layer_times': {},  # layer_num -> timestamp
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
                'final_detections': 0,
                'max_layer': 0,
            }
        
        elif event.event_name == 'yolo_layer_end':
            layer_num = event.fields.get('layer_num', 0)
            current_image['layer_times'][layer_num] = event.timestamp
            current_image['max_layer'] = max(current_image['max_layer'], layer_num)
        
        elif event.event_name == 'yolo_result':
            # Result after each layer (batch_size=1 means one layer at a time)
            # This contains detection count after processing this many layers
            processed_layers = event.fields.get('processed_layers', 0)
            total_detections = event.fields.get('total_detections', 0)
            
            # Store detection count for this layer
            current_image['layer_detections'][processed_layers] = total_detections
            
            # Keep track of the final (maximum) detection count
            # The last yolo_result will have the final count
            current_image['final_detections'] = total_detections
            current_image['max_layer'] = max(current_image['max_layer'], processed_layers)
    
    # Don't forget the last image
    if current_image['layer_detections']:
        images.append(current_image)
    
    return images


def calculate_quality_metrics(images):
    """
    Calculate quality metrics from image data
    """
    print("\n  Calculating quality metrics...")
    
    metrics = {
        'total_images': len(images),
        'layer_detection_progression': defaultdict(list),  # layer -> [detection counts]
        'layer_quality_ratio': defaultdict(list),  # layer -> [detection/final ratio]
        'sufficient_quality_layers': [],  # per image: first layer with sufficient quality
    }
    
    for img in images:
        final_count = img['final_detections']
        if final_count == 0:
            continue
        
        # Track progression
        for layer_num in sorted(img['layer_detections'].keys()):
            detections = img['layer_detections'][layer_num]
            metrics['layer_detection_progression'][layer_num].append(detections)
            
            # Quality ratio (how close to final result)
            ratio = detections / final_count if final_count > 0 else 0
            metrics['layer_quality_ratio'][layer_num].append(ratio)
        
        # Find first layer where we reach sufficient quality (e.g., 95% of final)
        for layer_num in sorted(img['layer_detections'].keys()):
            detections = img['layer_detections'][layer_num]
            ratio = detections / final_count if final_count > 0 else 0
            
            if ratio >= 0.95:  # 95% threshold
                metrics['sufficient_quality_layers'].append(layer_num)
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
    
    return metrics


def determine_cancellation_thresholds(metrics):
    """
    Determine recommended cancellation thresholds
    """
    print("\n  Determining cancellation thresholds...")
    
    thresholds = {
        '90%': None,
        '95%': None,
        '99%': None,
    }
    
    # Find first layer where mean quality ratio reaches each threshold
    for layer_num in sorted(metrics['avg_quality_ratio_per_layer'].keys()):
        mean_ratio = metrics['avg_quality_ratio_per_layer'][layer_num]['mean']
        
        if thresholds['90%'] is None and mean_ratio >= 0.90:
            thresholds['90%'] = layer_num
        if thresholds['95%'] is None and mean_ratio >= 0.95:
            thresholds['95%'] = layer_num
        if thresholds['99%'] is None and mean_ratio >= 0.99:
            thresholds['99%'] = layer_num
    
    # Calculate statistics on when images reach 95% quality
    if metrics['sufficient_quality_layers']:
        sufficient_stats = {
            'mean': np.mean(metrics['sufficient_quality_layers']),
            'median': np.median(metrics['sufficient_quality_layers']),
            'std': np.std(metrics['sufficient_quality_layers']),
            'min': np.min(metrics['sufficient_quality_layers']),
            'max': np.max(metrics['sufficient_quality_layers']),
        }
    else:
        sufficient_stats = None
    
    return thresholds, sufficient_stats


def plot_detection_progression(metrics):
    """
    Plot detection count progression across layers
    """
    print("\n  Creating detection progression plot...")
    
    layers = sorted(metrics['layer_detection_progression'].keys())
    means = [np.mean(metrics['layer_detection_progression'][l]) for l in layers]
    stds = [np.std(metrics['layer_detection_progression'][l]) for l in layers]
    
    fig, ax = plt.subplots(figsize=(12, 6))
    
    ax.errorbar(layers, means, yerr=stds, marker='o', capsize=5, linewidth=2, markersize=8)
    ax.set_xlabel('Layer Number')
    ax.set_ylabel('Average Detection Count')
    ax.set_title('YOLO Detection Count Progression Across Layers')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(QUALITY_DIR / 'detection_progression.png', dpi=300)
    plt.close()
    print(f"    Saved: {QUALITY_DIR / 'detection_progression.png'}")


def plot_quality_ratio_progression(metrics):
    """
    Plot quality ratio (detections/final) progression across layers
    """
    print("\n  Creating quality ratio progression plot...")
    
    layers = sorted(metrics['layer_quality_ratio'].keys())
    means = [np.mean(metrics['layer_quality_ratio'][l]) for l in layers]
    stds = [np.std(metrics['layer_quality_ratio'][l]) for l in layers]
    
    fig, ax = plt.subplots(figsize=(12, 6))
    
    ax.errorbar(layers, means, yerr=stds, marker='o', capsize=5, linewidth=2, markersize=8)
    ax.axhline(y=0.90, color='g', linestyle='--', label='90% Quality')
    ax.axhline(y=0.95, color='orange', linestyle='--', label='95% Quality')
    ax.axhline(y=0.99, color='r', linestyle='--', label='99% Quality')
    
    ax.set_xlabel('Layer Number')
    ax.set_ylabel('Quality Ratio (Detections / Final)')
    ax.set_title('YOLO Detection Quality Progression Across Layers')
    ax.set_ylim([0, 1.05])
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(QUALITY_DIR / 'quality_ratio_progression.png', dpi=300)
    plt.close()
    print(f"    Saved: {QUALITY_DIR / 'quality_ratio_progression.png'}")


def plot_cancellation_histogram(metrics):
    """
    Plot histogram of when images reach sufficient quality (95%)
    """
    print("\n  Creating cancellation histogram...")
    
    if not metrics['sufficient_quality_layers']:
        print("    No data for histogram")
        return
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    ax.hist(metrics['sufficient_quality_layers'], bins=20, edgecolor='black', alpha=0.7)
    ax.axvline(np.mean(metrics['sufficient_quality_layers']), 
               color='r', linestyle='--', linewidth=2, label=f"Mean: {np.mean(metrics['sufficient_quality_layers']):.1f}")
    ax.axvline(np.median(metrics['sufficient_quality_layers']), 
               color='g', linestyle='--', linewidth=2, label=f"Median: {np.median(metrics['sufficient_quality_layers']):.1f}")
    
    ax.set_xlabel('Layer Number')
    ax.set_ylabel('Number of Images')
    ax.set_title('Distribution: After How Many Layers Can We Cancel? (95% Quality)')
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig(QUALITY_DIR / 'cancellation_histogram.png', dpi=300)
    plt.close()
    print(f"    Saved: {QUALITY_DIR / 'cancellation_histogram.png'}")


def plot_layer_wise_boxplot(metrics):
    """
    Create box plot showing quality distribution at each layer
    """
    print("\n  Creating layer-wise quality boxplot...")
    
    layers = sorted(metrics['layer_quality_ratio'].keys())
    data = [metrics['layer_quality_ratio'][l] for l in layers]
    
    fig, ax = plt.subplots(figsize=(14, 6))
    
    bp = ax.boxplot(data, labels=layers, patch_artist=True)
    
    # Color boxes
    for patch in bp['boxes']:
        patch.set_facecolor('lightblue')
    
    # Add threshold lines
    ax.axhline(y=0.90, color='g', linestyle='--', alpha=0.5, label='90% Quality')
    ax.axhline(y=0.95, color='orange', linestyle='--', alpha=0.5, label='95% Quality')
    ax.axhline(y=0.99, color='r', linestyle='--', alpha=0.5, label='99% Quality')
    
    ax.set_xlabel('Layer Number')
    ax.set_ylabel('Quality Ratio')
    ax.set_title('Quality Distribution at Each Layer')
    ax.set_ylim([0, 1.05])
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig(QUALITY_DIR / 'quality_boxplot.png', dpi=300)
    plt.close()
    print(f"    Saved: {QUALITY_DIR / 'quality_boxplot.png'}")


def export_quality_results(metrics, thresholds, sufficient_stats):
    """
    Export quality analysis results to JSON and text
    """
    print("\n  Exporting results...")
    
    # Create summary report
    report = {
        'analysis_date': datetime.now().isoformat(),
        'total_images_analyzed': metrics['total_images'],
        'cancellation_thresholds': thresholds,
        'sufficient_quality_stats': sufficient_stats,
        'layer_quality_progression': {
            int(k): v for k, v in metrics['avg_quality_ratio_per_layer'].items()
        },
        'layer_detection_counts': {
            int(k): v for k, v in metrics['avg_detections_per_layer'].items()
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
        
        f.write(f"Analysis Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Total Images Analyzed: {metrics['total_images']}\n\n")
        
        f.write("RECOMMENDED CANCELLATION THRESHOLDS:\n")
        f.write("-" * 80 + "\n")
        for quality, layer in thresholds.items():
            if layer:
                f.write(f"  {quality} Quality: Cancel after layer {layer}\n")
            else:
                f.write(f"  {quality} Quality: Never reached\n")
        f.write("\n")
        
        if sufficient_stats:
            f.write("95% QUALITY ACHIEVEMENT STATISTICS:\n")
            f.write("-" * 80 + "\n")
            f.write(f"  Mean:   {sufficient_stats['mean']:.2f} layers\n")
            f.write(f"  Median: {sufficient_stats['median']:.2f} layers\n")
            f.write(f"  Std:    {sufficient_stats['std']:.2f} layers\n")
            f.write(f"  Min:    {sufficient_stats['min']} layers\n")
            f.write(f"  Max:    {sufficient_stats['max']} layers\n")
            f.write("\n")
        
        f.write("QUALITY PROGRESSION BY LAYER:\n")
        f.write("-" * 80 + "\n")
        for layer in sorted(metrics['avg_quality_ratio_per_layer'].keys()):
            stats = metrics['avg_quality_ratio_per_layer'][layer]
            f.write(f"  Layer {layer:2d}: {stats['mean']:.3f} ± {stats['std']:.3f} ")
            f.write(f"(min: {stats['min']:.3f}, max: {stats['max']:.3f})\n")
        
        f.write("\n" + "="*80 + "\n")
    
    print(f"    Saved: {summary_path}")
    
    # Print to console
    with open(summary_path, 'r') as f:
        print("\n" + f.read())


def main():
    """Main quality analysis function"""
    print("="*80)
    print("YOLO QUALITY ANALYSIS (Phase 2)")
    print("="*80)
    print(f"Trace directory: {TRACE_DIR}")
    print(f"Quality results: {QUALITY_DIR}")
    
    # Find Phase 1 baseline traces
    phase1_traces = [d for d in TRACE_DIR.iterdir() 
                     if d.is_dir() and ('phase1_baseline' in d.name or 'baseline' in d.name)]
    
    if not phase1_traces:
        print("\nError: No Phase 1 baseline traces found!")
        print(f"Please run Phase 1 experiments first: ./run_phase1_baseline.sh")
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
    thresholds, sufficient_stats = determine_cancellation_thresholds(metrics)
    
    # Generate plots
    print("\nGenerating plots...")
    plot_detection_progression(metrics)
    plot_quality_ratio_progression(metrics)
    plot_cancellation_histogram(metrics)
    plot_layer_wise_boxplot(metrics)
    
    # Export results
    export_quality_results(metrics, thresholds, sufficient_stats)
    
    print("\n" + "="*80)
    print("QUALITY ANALYSIS COMPLETE")
    print("="*80)
    print(f"Results saved to: {QUALITY_DIR}")
    print(f"Plots saved to: {QUALITY_DIR}")
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
