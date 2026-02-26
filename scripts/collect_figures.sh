#!/bin/bash
# Collect paper figures and tables into paper_figures/ for easy reviewer access
#
# Usage: ./scripts/collect_figures.sh
#
# Copies only the figures and tables that appear in the paper.
# Files are prefixed with their paper reference (e.g., figure_5a_, table_1_).
# Only copies files that exist — safe to run after partial experiments.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
OUTPUT_DIR="${WORKSPACE_DIR}/paper_figures"

MC_DIR="${WORKSPACE_DIR}/experiments/monte_carlo"
IF_DIR="${WORKSPACE_DIR}/experiments/interference"
YOLO_DIR="${WORKSPACE_DIR}/experiments/yolo"

mkdir -p "${OUTPUT_DIR}"

collected=0

copy_if_exists() {
    local src="$1"
    local dst="$2"
    if [ -f "${src}" ]; then
        cp "${src}" "${dst}"
        echo "  Collected: $(basename "${dst}")"
        collected=$((collected + 1))
    fi
}

echo "Collecting paper figures into ${OUTPUT_DIR}/ ..."
echo ""

# Figure 5a: Monte Carlo — total iterations
copy_if_exists "${MC_DIR}/results/plots/total_iterations.pdf" \
               "${OUTPUT_DIR}/figure_5a_total_iterations.pdf"

# Figure 5b: Monte Carlo — cancel-to-finish latency
copy_if_exists "${MC_DIR}/results/plots/cancel_to_finish_latency.pdf" \
               "${OUTPUT_DIR}/figure_5b_cancel_to_finish_latency.pdf"

# Figure 6a: Interference — compute time vs batch size
copy_if_exists "${IF_DIR}/results/plots/compute_time_vs_batch_size.pdf" \
               "${OUTPUT_DIR}/figure_6a_compute_time_vs_batch_size.pdf"

# Figure 6b: Interference — timer period vs batch size
copy_if_exists "${IF_DIR}/results/plots/timer_period_vs_batch_size.pdf" \
               "${OUTPUT_DIR}/figure_6b_timer_period_vs_batch_size.pdf"

# Table I: Interference — missed timer periods
copy_if_exists "${IF_DIR}/results/table_1_missed_periods.csv" \
               "${OUTPUT_DIR}/table_1_interference_metrics.csv"

# Figure 7a: YOLO — quality ratio progression
copy_if_exists "${YOLO_DIR}/results/quality_analysis/quality_ratio_progression.pdf" \
               "${OUTPUT_DIR}/figure_7a_quality_ratio_progression.pdf"

# Figure 7b: YOLO — total runtime comparison
copy_if_exists "${YOLO_DIR}/results/phase4_analysis/total_runtime_comparison.pdf" \
               "${OUTPUT_DIR}/figure_7b_total_runtime_comparison.pdf"

echo ""
if [ ${collected} -eq 0 ]; then
    echo "No figures found. Run experiments first, then re-run this script."
else
    echo "Collected ${collected} file(s) in ${OUTPUT_DIR}/"
fi
