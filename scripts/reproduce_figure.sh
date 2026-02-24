#!/bin/bash
# Reproduce a specific paper figure
#
# Paper: "Anytime ROS 2: Timely Task Completion in Non-Preemptive Robotic Systems"
#
# Usage: ./scripts/reproduce_figure.sh <figure> [--quick]
#
# Figures:
#   5a    Fig 5a: Monte Carlo segment count vs batch size     (CPU, ~40m / ~5m quick)
#   5b    Fig 5b: Monte Carlo cancellation delay              (CPU, ~40m / ~5m quick)
#   5     Both Fig 5a + 5b                                    (CPU, ~40m / ~5m quick)
#   6     Fig 6a+6b + Table I: Interference results           (CPU, ~40m / ~3m quick)
#   7a    Fig 7a: YOLO quality progression                    (GPU, ~30m)
#   7b    Fig 7b: YOLO runtime comparison                     (GPU, ~1h)
#   7     Both Fig 7a + 7b                                    (GPU, ~1.5h)
#
# Options:
#   --quick   Use quick experiments for CPU figures (fewer batch sizes, shorter runs)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
PACKAGES_DIR="${WORKSPACE_DIR}/packages"
MC_DIR="${WORKSPACE_DIR}/experiments/monte_carlo"
IF_DIR="${WORKSPACE_DIR}/experiments/interference"
YOLO_DIR="${WORKSPACE_DIR}/experiments/yolo"

# Parse arguments
FIGURE="${1:-}"
shift || true

USE_QUICK=false
for arg in "$@"; do
    case $arg in
        --quick) USE_QUICK=true ;;
    esac
done

print_usage() {
    echo "Usage: $0 <figure> [--quick]"
    echo ""
    echo "Figures:"
    echo "  5a    Fig 5a: Monte Carlo segment count          (CPU)"
    echo "  5b    Fig 5b: Monte Carlo cancellation delay     (CPU)"
    echo "  5     Both Fig 5a + 5b                           (CPU)"
    echo "  6     Fig 6a+6b + Table I: Interference          (CPU)"
    echo "  7a    Fig 7a: YOLO quality progression           (GPU)"
    echo "  7b    Fig 7b: YOLO runtime comparison            (GPU)"
    echo "  7     Both Fig 7a + 7b                           (GPU)"
    echo ""
    echo "Options:"
    echo "  --quick   Quick mode for CPU figures (fewer configs, shorter runs)"
    exit 1
}

if [ -z "${FIGURE}" ]; then
    print_usage
fi

# Source workspace
cd "${PACKAGES_DIR}"
source install/setup.bash

# ─────────────────────────────────────────────
# Monte Carlo experiments (Figures 5a, 5b)
# ─────────────────────────────────────────────
run_monte_carlo() {
    echo "========================================="
    echo "Reproducing Monte Carlo results"
    echo "========================================="
    echo ""

    if [ "${USE_QUICK}" = true ]; then
        echo "Mode: quick (3 batch sizes, 5s runs)"
        "${MC_DIR}/run_quick.sh"
    else
        echo "Mode: full (7 batch sizes, 10s runs)"
        "${MC_DIR}/run_monte_carlo_experiments.sh"
    fi

    echo ""
    echo "========================================="
    echo "Output files:"
    echo "========================================="
    MC_PLOTS="${MC_DIR}/results/plots"
    case "${FIGURE}" in
        5a)
            echo "  Figure 5a: ${MC_PLOTS}/batch_size_vs_iterations.pdf"
            ;;
        5b)
            echo "  Figure 5b: ${MC_PLOTS}/cancellation_delay.pdf"
            ;;
        5)
            echo "  Figure 5a: ${MC_PLOTS}/batch_size_vs_iterations.pdf"
            echo "  Figure 5b: ${MC_PLOTS}/cancellation_delay.pdf"
            ;;
    esac
    echo "  All plots: ${MC_PLOTS}/"
    echo "  CSV data:  ${MC_DIR}/results/aggregated_results.csv"
}

# ─────────────────────────────────────────────
# Interference experiments (Figure 6)
# ─────────────────────────────────────────────
run_interference() {
    echo "========================================="
    echo "Reproducing Interference results"
    echo "========================================="
    echo ""

    if [ "${USE_QUICK}" = true ]; then
        echo "Mode: quick (3 batch sizes, 5s runs)"
        "${IF_DIR}/run_quick.sh"
    else
        echo "Mode: full (7 batch sizes, 10s runs)"
        "${IF_DIR}/run_interference_experiments.sh"
    fi

    echo ""
    echo "========================================="
    echo "Output files:"
    echo "========================================="
    IF_PLOTS="${IF_DIR}/results/plots"
    echo "  Figure 6a: ${IF_PLOTS}/jitter_vs_batch_size.pdf"
    echo "  Figure 6b: ${IF_PLOTS}/compute_time_vs_batch_size.pdf"
    echo "  Table I:   ${IF_DIR}/results/aggregated_results.csv"
    echo "  All plots: ${IF_PLOTS}/"
}

# ─────────────────────────────────────────────
# YOLO quality (Figure 7a)
# ─────────────────────────────────────────────
run_yolo_quality() {
    echo "========================================="
    echo "Reproducing YOLO quality progression"
    echo "========================================="
    echo ""

    # Check GPU
    if ! command -v nvidia-smi &>/dev/null || ! nvidia-smi &>/dev/null 2>&1; then
        echo "ERROR: GPU required for YOLO experiments but nvidia-smi not available"
        exit 1
    fi

    echo "Step 0: Generating baseline configs..."
    python3 "${YOLO_DIR}/0_generate_baseline_configs.py"

    echo "Step 1: Collecting baseline data (~30 min)..."
    "${YOLO_DIR}/1_collect_baseline.sh"

    echo "Step 2a: Analyzing quality..."
    python3 "${YOLO_DIR}/2a_analyze_quality.py"

    echo ""
    echo "========================================="
    echo "Output files:"
    echo "========================================="
    echo "  Figure 7a: ${YOLO_DIR}/results/quality_analysis/quality_ratio_progression.pdf"
    echo "  All plots: ${YOLO_DIR}/results/quality_analysis/"
}

# ─────────────────────────────────────────────
# YOLO throughput (Figure 7b)
# ─────────────────────────────────────────────
run_yolo_throughput() {
    echo "========================================="
    echo "Reproducing YOLO runtime comparison"
    echo "========================================="
    echo ""

    # Check GPU
    if ! command -v nvidia-smi &>/dev/null || ! nvidia-smi &>/dev/null 2>&1; then
        echo "ERROR: GPU required for YOLO experiments but nvidia-smi not available"
        exit 1
    fi

    echo "Step 0: Generating baseline configs..."
    python3 "${YOLO_DIR}/0_generate_baseline_configs.py"

    echo "Step 1: Collecting baseline data (~30 min)..."
    "${YOLO_DIR}/1_collect_baseline.sh"

    echo "Step 3: Measuring throughput (~30 min)..."
    "${YOLO_DIR}/3_measure_throughput.sh"

    echo "Step 4: Analyzing throughput..."
    python3 "${YOLO_DIR}/4_analyze_throughput.py"

    echo ""
    echo "========================================="
    echo "Output files:"
    echo "========================================="
    echo "  Figure 7b: ${YOLO_DIR}/results/runtime_analysis/throughput_comparison.png"
    echo "  All plots: ${YOLO_DIR}/results/runtime_analysis/"
}

# ─────────────────────────────────────────────
# YOLO all (Figures 7a + 7b)
# ─────────────────────────────────────────────
run_yolo_all() {
    echo "========================================="
    echo "Reproducing all YOLO results"
    echo "========================================="
    echo ""

    # Check GPU
    if ! command -v nvidia-smi &>/dev/null || ! nvidia-smi &>/dev/null 2>&1; then
        echo "ERROR: GPU required for YOLO experiments but nvidia-smi not available"
        exit 1
    fi

    echo "Step 0: Generating baseline configs..."
    python3 "${YOLO_DIR}/0_generate_baseline_configs.py"

    echo "Step 1: Collecting baseline data (~30 min)..."
    "${YOLO_DIR}/1_collect_baseline.sh"

    echo "Step 2a: Analyzing quality..."
    python3 "${YOLO_DIR}/2a_analyze_quality.py"

    echo "Step 2b: Analyzing block sizes..."
    python3 "${YOLO_DIR}/2b_analyze_blocks.py"

    echo "Step 3: Measuring throughput (~30 min)..."
    "${YOLO_DIR}/3_measure_throughput.sh"

    echo "Step 4: Analyzing throughput..."
    python3 "${YOLO_DIR}/4_analyze_throughput.py"

    echo ""
    echo "========================================="
    echo "Output files:"
    echo "========================================="
    echo "  Figure 7a: ${YOLO_DIR}/results/quality_analysis/quality_ratio_progression.pdf"
    echo "  Figure 7b: ${YOLO_DIR}/results/runtime_analysis/throughput_comparison.png"
    echo "  Quality:   ${YOLO_DIR}/results/quality_analysis/"
    echo "  Blocks:    ${YOLO_DIR}/results/block_analysis/"
    echo "  Runtime:   ${YOLO_DIR}/results/runtime_analysis/"
}

# ─────────────────────────────────────────────
# Dispatch
# ─────────────────────────────────────────────
case "${FIGURE}" in
    5a|5b|5)
        run_monte_carlo
        ;;
    6)
        run_interference
        ;;
    7a)
        run_yolo_quality
        ;;
    7b)
        run_yolo_throughput
        ;;
    7)
        run_yolo_all
        ;;
    *)
        echo "Unknown figure: ${FIGURE}"
        echo ""
        print_usage
        ;;
esac

echo ""
echo "Done."
