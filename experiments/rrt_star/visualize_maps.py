#!/usr/bin/env python3
"""
Visualize RRT* experiment maps with start and goal positions.

Loads Nav2 PGM+YAML maps and overlays the configured start/goal markers.
Produces one PNG/PDF per map showing the occupancy grid in world coordinates.

Usage:
    python3 visualize_maps.py [--output-dir DIR] [--format png|pdf]
"""

import argparse
import struct
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np


# Default map configurations (matching evaluation_plan.md Section 4.1)
MAP_CONFIGS = {
    "depot": {
        "start": (5.0, 5.0),
        "goal": (25.0, 10.0),
    },
    "warehouse": {
        "start": (-5.0, -20.0),
        "goal": (5.0, 20.0),
    },
}

# Where to find maps (relative to this script)
SCRIPT_DIR = Path(__file__).resolve().parent
MAPS_DIR = SCRIPT_DIR / ".." / ".." / "packages" / "src" / "anytime_rrt_star" / "maps"


def parse_yaml_flat(yaml_path):
    """Parse a simple flat YAML file (no nested structures)."""
    params = {}
    with open(yaml_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            if ':' in line:
                key, value = line.split(':', 1)
                key = key.strip()
                value = value.strip()
                # Parse list values like [0.0, 0.0, 0]
                if value.startswith('[') and value.endswith(']'):
                    inner = value[1:-1]
                    params[key] = [float(v.strip()) for v in inner.split(',')]
                elif value.replace('.', '').replace('-', '').isdigit():
                    params[key] = float(value) if '.' in value else int(value)
                else:
                    params[key] = value
    return params


def load_pgm_binary(pgm_path):
    """Load a binary (P5) PGM file, returning (width, height, pixels as 2D numpy array)."""
    with open(pgm_path, 'rb') as f:
        # Read magic number
        magic = f.readline().decode().strip()
        if magic != 'P5':
            raise ValueError(f"Expected P5 PGM, got {magic}")

        # Skip comments
        line = f.readline().decode().strip()
        while line.startswith('#'):
            line = f.readline().decode().strip()

        # Read dimensions
        width, height = map(int, line.split())

        # Read max value
        max_val = int(f.readline().decode().strip())

        # Read pixel data
        if max_val <= 255:
            data = f.read(width * height)
            pixels = np.frombuffer(data, dtype=np.uint8).reshape((height, width))
        else:
            data = f.read(width * height * 2)
            pixels = np.frombuffer(data, dtype='>u2').reshape((height, width))

    return width, height, max_val, pixels


def build_occupancy_image(pixels, max_val, occupied_thresh, free_thresh, negate):
    """
    Convert raw PGM pixels to a colored occupancy image (RGB).

    Returns an (H, W, 3) uint8 array:
      - White  (255,255,255) for free space
      - Black  (0,0,0)       for occupied
      - Gray   (128,128,128) for unknown
    """
    # Compute occupancy probability per Nav2 convention
    occ_prob = (max_val - pixels.astype(float)) / max_val
    if negate:
        occ_prob = 1.0 - occ_prob

    h, w = pixels.shape
    img = np.full((h, w, 3), 128, dtype=np.uint8)  # default: unknown (gray)

    free_mask = occ_prob < free_thresh
    occupied_mask = occ_prob > occupied_thresh

    img[free_mask] = [255, 255, 255]      # free = white
    img[occupied_mask] = [0, 0, 0]        # occupied = black

    return img


def visualize_map(yaml_path, map_name, config, output_dir, fmt='png'):
    """Visualize a single map with start/goal positions."""
    params = parse_yaml_flat(yaml_path)

    pgm_path = yaml_path.parent / params['image']
    if not pgm_path.exists():
        print(f"  ERROR: PGM file not found: {pgm_path}")
        return

    width, height, max_val, pixels = load_pgm_binary(pgm_path)
    resolution = params['resolution']
    origin = params.get('origin', [0.0, 0.0, 0.0])
    origin_x, origin_y = origin[0], origin[1]
    negate = bool(params.get('negate', 0))
    occupied_thresh = params.get('occupied_thresh', 0.65)
    free_thresh = params.get('free_thresh', 0.25)

    print(f"  Map: {map_name} ({width}x{height}, resolution={resolution}m)")

    # Build occupancy image
    occ_img = build_occupancy_image(pixels, max_val, occupied_thresh, free_thresh, negate)

    # Flip vertically: PGM row 0 is top, but map origin is bottom-left
    occ_img = np.flipud(occ_img)

    # Compute world-coordinate extent
    x_min = origin_x
    x_max = origin_x + width * resolution
    y_min = origin_y
    y_max = origin_y + height * resolution

    # Plot
    fig, ax = plt.subplots(figsize=(14, 10))

    ax.imshow(occ_img, extent=[x_min, x_max, y_min, y_max],
              origin='lower', aspect='equal', interpolation='nearest')

    # Draw start and goal
    start = config['start']
    goal = config['goal']

    marker_size = 250
    ax.scatter(*start, c='limegreen', s=marker_size, marker='o',
               edgecolors='black', linewidths=2, zorder=10, label='Start')
    ax.scatter(*goal, c='red', s=marker_size, marker='*',
               edgecolors='black', linewidths=1.5, zorder=10, label='Goal')

    # Annotate with coordinates
    offset = max((x_max - x_min), (y_max - y_min)) * 0.02
    ax.annotate(f'Start ({start[0]}, {start[1]})',
                xy=start, xytext=(start[0] + offset, start[1] + offset),
                fontsize=12, fontweight='bold', color='darkgreen',
                arrowprops=dict(arrowstyle='->', color='darkgreen', lw=1.5))
    ax.annotate(f'Goal ({goal[0]}, {goal[1]})',
                xy=goal, xytext=(goal[0] + offset, goal[1] + offset),
                fontsize=12, fontweight='bold', color='darkred',
                arrowprops=dict(arrowstyle='->', color='darkred', lw=1.5))

    ax.set_xlabel('X (meters)', fontsize=14)
    ax.set_ylabel('Y (meters)', fontsize=14)
    ax.set_title(f'{map_name.capitalize()} Map — Start & Goal Positions', fontsize=16)
    ax.legend(fontsize=13, loc='best')
    ax.tick_params(labelsize=12)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = output_dir / f'map_{map_name}.{fmt}'
    plt.savefig(out_path, dpi=200, bbox_inches='tight')
    plt.close()
    print(f"  Saved: {out_path}")


def main():
    parser = argparse.ArgumentParser(description='Visualize RRT* maps with start/goal positions')
    parser.add_argument('--output-dir', type=str, default=None,
                        help='Output directory for images (default: experiments/rrt_star/results/plots)')
    parser.add_argument('--format', type=str, default='png', choices=['png', 'pdf'],
                        help='Output image format')
    parser.add_argument('--maps-dir', type=str, default=None,
                        help='Directory containing map YAML+PGM files')
    args = parser.parse_args()

    maps_dir = Path(args.maps_dir) if args.maps_dir else MAPS_DIR
    output_dir = Path(args.output_dir) if args.output_dir else (SCRIPT_DIR / "results" / "plots")
    output_dir.mkdir(parents=True, exist_ok=True)

    print("========================================")
    print("RRT* Map Visualization")
    print("========================================")
    print(f"Maps directory: {maps_dir}")
    print(f"Output directory: {output_dir}")
    print(f"Format: {args.format}")
    print()

    for map_name, config in MAP_CONFIGS.items():
        yaml_path = maps_dir / f"{map_name}.yaml"
        if not yaml_path.exists():
            print(f"  WARNING: {yaml_path} not found, skipping {map_name}")
            continue
        visualize_map(yaml_path, map_name, config, output_dir, args.format)

    print("\nDone!")
    return 0


if __name__ == "__main__":
    sys.exit(main())
