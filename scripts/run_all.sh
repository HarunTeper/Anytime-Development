#!/bin/bash
# End-to-end artifact evaluation runner
# Runs all experiments and generates analysis plots
#
# Usage: ./scripts/run_all.sh [--quick|--full] [--cpu-only]
#
# Flags:
#   --quick     Use quick experiment scripts (3 batch sizes, 5s runs) [default]
#   --full      Use full experiment scripts (7 batch sizes, 10s runs)
#   --cpu-only  Skip YOLO experiments (which require GPU)
#
# Duration estimates:
#   --quick --cpu-only  ~10 minutes
#   --full  --cpu-only  ~1.5 hours
#   --full              ~5+ hours (includes YOLO pipeline)

set -e

# Cleanup on interrupt
cleanup() {
    echo ""
    echo "Interrupted — cleaning up..."
    lttng stop 2>/dev/null || true
    lttng destroy 2>/dev/null || true
    pkill -9 -f 'anytime_monte_carlo' 2>/dev/null || true
    pkill -9 -f 'interference' 2>/dev/null || true
    pkill -9 -f 'ros2' 2>/dev/null || true
}
trap cleanup INT TERM

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
PACKAGES_DIR="${WORKSPACE_DIR}/packages"

# Parse flags
MODE="quick"
CPU_ONLY=false

for arg in "$@"; do
    case $arg in
        --quick) MODE="quick" ;;
        --full)  MODE="full" ;;
        --cpu-only) CPU_ONLY=true ;;
        --help|-h)
            echo "Usage: $0 [--quick|--full] [--cpu-only]"
            echo ""
            echo "Flags:"
            echo "  --quick     Quick experiments (3 batch sizes, 5s) [default]"
            echo "  --full      Full experiments (7 batch sizes, 10s)"
            echo "  --cpu-only  Skip YOLO experiments (require GPU)"
            exit 0
            ;;
        *)
            echo "Unknown option: $arg"
            echo "Usage: $0 [--quick|--full] [--cpu-only]"
            exit 1
            ;;
    esac
done

# GPU detection
HAS_GPU=false
if command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null 2>&1; then
    HAS_GPU=true
fi

echo "========================================="
echo "Anytime ROS 2 — Full Evaluation"
echo "========================================="
echo ""
echo "Mode:      ${MODE}"
echo "CPU only:  ${CPU_ONLY}"
echo "GPU:       ${HAS_GPU}"
echo "Workspace: ${WORKSPACE_DIR}"
echo ""

passed=0
failed=0
skipped=0

run_phase() {
    local name="$1"
    local cmd="$2"
    echo ""
    echo "-----------------------------------------"
    echo "Phase: ${name}"
    echo "-----------------------------------------"
    if eval "${cmd}"; then
        echo "  PASSED: ${name}"
        passed=$((passed + 1))
    else
        echo "  FAILED: ${name}"
        failed=$((failed + 1))
    fi
}

skip_phase() {
    local name="$1"
    local reason="$2"
    echo ""
    echo "-----------------------------------------"
    echo "Phase: ${name} — SKIPPED (${reason})"
    echo "-----------------------------------------"
    skipped=$((skipped + 1))
}

# ─────────────────────────────────────────────
# Phase 1: Smoke test
# ─────────────────────────────────────────────
run_phase "Smoke test" "${SCRIPT_DIR}/smoke_test.sh"

# ─────────────────────────────────────────────
# Phase 2: Monte Carlo experiments
# ─────────────────────────────────────────────
if [ "${MODE}" = "quick" ]; then
    run_phase "Monte Carlo experiments (quick)" "${WORKSPACE_DIR}/experiments/monte_carlo/run_quick.sh"
else
    run_phase "Monte Carlo experiments (full)" "${WORKSPACE_DIR}/experiments/monte_carlo/run_monte_carlo_experiments.sh"
fi

# ─────────────────────────────────────────────
# Phase 3: Interference experiments
# ─────────────────────────────────────────────
if [ "${MODE}" = "quick" ]; then
    run_phase "Interference experiments (quick)" "${WORKSPACE_DIR}/experiments/interference/run_quick.sh"
else
    run_phase "Interference experiments (full)" "${WORKSPACE_DIR}/experiments/interference/run_interference_experiments.sh"
fi

# ─────────────────────────────────────────────
# Phase 4: YOLO experiments (GPU only, full only)
# ─────────────────────────────────────────────
if [ "${CPU_ONLY}" = true ]; then
    skip_phase "YOLO experiments" "--cpu-only flag set"
elif [ "${HAS_GPU}" = false ]; then
    skip_phase "YOLO experiments" "no GPU detected"
elif [ "${MODE}" = "quick" ]; then
    skip_phase "YOLO experiments" "quick mode (YOLO requires full mode)"
else
    YOLO_DIR="${WORKSPACE_DIR}/experiments/yolo"

    echo ""
    echo "-----------------------------------------"
    echo "Phase: YOLO experiments (full pipeline)"
    echo "-----------------------------------------"

    cd "${PACKAGES_DIR}"
    source install/setup.bash

    yolo_failed=false

    echo "  Step 0: Generating baseline configs..."
    python3 "${YOLO_DIR}/0_generate_baseline_configs.py" || yolo_failed=true

    if [ "${yolo_failed}" = false ]; then
        echo "  Step 1: Collecting baseline data..."
        "${YOLO_DIR}/1_collect_baseline.sh" || yolo_failed=true
    fi

    if [ "${yolo_failed}" = false ]; then
        echo "  Step 2a: Analyzing quality..."
        python3 "${YOLO_DIR}/2a_analyze_quality.py" || yolo_failed=true
    fi

    if [ "${yolo_failed}" = false ]; then
        echo "  Step 2b: Analyzing block sizes..."
        python3 "${YOLO_DIR}/2b_analyze_blocks.py" || yolo_failed=true
    fi

    if [ "${yolo_failed}" = false ]; then
        echo "  Step 3: Measuring throughput..."
        "${YOLO_DIR}/3_measure_throughput.sh" || yolo_failed=true
    fi

    if [ "${yolo_failed}" = false ]; then
        echo "  Step 4: Analyzing throughput..."
        python3 "${YOLO_DIR}/4_analyze_throughput.py" || yolo_failed=true
    fi

    if [ "${yolo_failed}" = false ]; then
        echo "  Step 5: Generating cancellation configs..."
        python3 "${YOLO_DIR}/5_generate_configs.py" || yolo_failed=true
    fi

    if [ "${yolo_failed}" = false ]; then
        echo "  Step 6: Running cancellation experiments..."
        "${YOLO_DIR}/6_run_experiments.sh" || yolo_failed=true
    fi

    if [ "${yolo_failed}" = false ]; then
        echo "  Step 7: Analyzing cancellation..."
        python3 "${YOLO_DIR}/7_analyze_cancellation.py" || yolo_failed=true
    fi

    if [ "${yolo_failed}" = false ]; then
        echo "  PASSED: YOLO experiments"
        passed=$((passed + 1))
    else
        echo "  FAILED: YOLO experiments"
        failed=$((failed + 1))
    fi
fi

# ─────────────────────────────────────────────
# Summary
# ─────────────────────────────────────────────
total=$((passed + failed + skipped))

echo ""
echo "========================================="
echo "Evaluation Summary"
echo "========================================="
echo "  Passed:  ${passed}"
echo "  Failed:  ${failed}"
echo "  Skipped: ${skipped}"
echo "  Total:   ${total}"
echo ""
echo "Result locations:"
echo "  Monte Carlo:   ${WORKSPACE_DIR}/experiments/monte_carlo/results/"
echo "  Interference:  ${WORKSPACE_DIR}/experiments/interference/results/"
if [ "${CPU_ONLY}" = false ] && [ "${HAS_GPU}" = true ] && [ "${MODE}" = "full" ]; then
    echo "  YOLO:          ${WORKSPACE_DIR}/experiments/yolo/results/"
fi
echo ""

if [ ${failed} -eq 0 ]; then
    echo "ALL PHASES PASSED"
    exit 0
else
    echo "SOME PHASES FAILED"
    exit 1
fi
