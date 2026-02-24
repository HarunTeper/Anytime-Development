#!/bin/bash
# End-to-end artifact evaluation runner
# Runs experiments and generates analysis plots
#
# Usage: ./scripts/run_all.sh [OPTIONS]
#
# Mode flags:
#   --quick           Use quick experiment scripts (3 batch sizes, 5s runs) [default]
#   --full            Use full experiment scripts (7 batch sizes, 10s runs)
#
# Experiment selection:
#   --cpu-only        Run only CPU experiments (Monte Carlo + Interference)
#   --gpu-only        Run only GPU experiments (YOLO)
#   --monte-carlo     Run Monte Carlo experiments
#   --interference    Run Interference experiments
#   --yolo            Run YOLO experiments
#
# Skip steps:
#   --no-smoke-test       Skip the smoke test
#   --no-prerequisites    Skip YOLO prerequisite checks (weights + images)
#   --no-build            Skip workspace build (smoke test includes build)
#   --no-figures          Skip paper figure collection
#
# Run individual steps only:
#   --only-smoke-test     Run only the smoke test
#   --only-prerequisites  Run only the YOLO prerequisite check
#   --only-build          Run only the workspace build
#   --only-figures        Run only paper figure collection
#
# If none of --monte-carlo, --interference, --yolo, --cpu-only, --gpu-only
# are specified, all experiments are run.
#
# Duration estimates:
#   --quick --cpu-only     ~10 minutes
#   --full  --cpu-only     ~1.5 hours
#   --full  --gpu-only     ~3 hours
#   --full                 ~5+ hours (all experiments)

set -e

# Cleanup on interrupt
cleanup() {
    echo ""
    echo "Interrupted — cleaning up..."
    lttng stop 2>/dev/null || true
    pkill -9 -f 'component_container' 2>/dev/null || true
    pkill -9 -f 'anytime_monte_carlo' 2>/dev/null || true
    pkill -9 -f 'interference_timer' 2>/dev/null || true
    pkill -9 -f 'ros2' 2>/dev/null || true
    sleep 1
    lttng destroy 2>/dev/null || true
}
trap cleanup INT TERM

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
PACKAGES_DIR="${WORKSPACE_DIR}/packages"

# Parse flags
MODE="quick"
CPU_ONLY=false
GPU_ONLY=false
RUN_MC=false
RUN_IF=false
RUN_YOLO=false
NO_SMOKE=false
NO_PREREQS=false
NO_BUILD=false
NO_FIGURES=false
ONLY_SMOKE=false
ONLY_PREREQS=false
ONLY_BUILD=false
ONLY_FIGURES=false
EXPLICIT_SELECTION=false

for arg in "$@"; do
    case $arg in
        --quick) MODE="quick" ;;
        --full)  MODE="full" ;;
        --cpu-only) CPU_ONLY=true ;;
        --gpu-only) GPU_ONLY=true ;;
        --monte-carlo) RUN_MC=true; EXPLICIT_SELECTION=true ;;
        --interference) RUN_IF=true; EXPLICIT_SELECTION=true ;;
        --yolo) RUN_YOLO=true; EXPLICIT_SELECTION=true ;;
        --no-smoke-test) NO_SMOKE=true ;;
        --no-prerequisites) NO_PREREQS=true ;;
        --no-build) NO_BUILD=true ;;
        --no-figures) NO_FIGURES=true ;;
        --only-smoke-test) ONLY_SMOKE=true ;;
        --only-prerequisites) ONLY_PREREQS=true ;;
        --only-build) ONLY_BUILD=true ;;
        --only-figures) ONLY_FIGURES=true ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Mode flags:"
            echo "  --quick               Quick experiments (3 batch sizes, 5s) [default]"
            echo "  --full                Full experiments (7 batch sizes, 10s)"
            echo ""
            echo "Experiment selection:"
            echo "  --cpu-only            Run only CPU experiments (Monte Carlo + Interference)"
            echo "  --gpu-only            Run only GPU experiments (YOLO)"
            echo "  --monte-carlo         Run Monte Carlo experiments"
            echo "  --interference        Run Interference experiments"
            echo "  --yolo                Run YOLO experiments"
            echo ""
            echo "Skip steps:"
            echo "  --no-smoke-test       Skip the smoke test"
            echo "  --no-prerequisites    Skip YOLO prerequisite checks"
            echo "  --no-build            Skip workspace build"
            echo "  --no-figures          Skip paper figure collection"
            echo ""
            echo "Run individual steps only:"
            echo "  --only-smoke-test     Run only the smoke test"
            echo "  --only-prerequisites  Run only the YOLO prerequisite check"
            echo "  --only-build          Run only the workspace build"
            echo "  --only-figures        Run only paper figure collection"
            echo ""
            echo "If no experiment flags are given, all experiments are run."
            echo "Flags can be combined: --monte-carlo --interference"
            echo ""
            echo "Duration estimates:"
            echo "  --quick --cpu-only     ~10 minutes"
            echo "  --full  --cpu-only     ~1.5 hours"
            echo "  --full  --gpu-only     ~3 hours"
            echo "  --full                 ~5+ hours"
            exit 0
            ;;
        *)
            echo "Unknown option: $arg"
            echo "Run '$0 --help' for usage."
            exit 1
            ;;
    esac
done

# ─────────────────────────────────────────────
# Handle --only-* flags (run single step and exit)
# ─────────────────────────────────────────────
if [ "${ONLY_BUILD}" = true ]; then
    echo "Building workspace..."
    cd "${PACKAGES_DIR}"
    colcon build --symlink-install
    echo "Build complete."
    exit 0
fi

if [ "${ONLY_PREREQS}" = true ]; then
    "${SCRIPT_DIR}/check_yolo_prerequisites.sh"
    exit 0
fi

if [ "${ONLY_SMOKE}" = true ]; then
    "${SCRIPT_DIR}/smoke_test.sh"
    exit 0
fi

if [ "${ONLY_FIGURES}" = true ]; then
    "${SCRIPT_DIR}/collect_figures.sh"
    exit 0
fi

# Validate mutually exclusive flags
if [ "${CPU_ONLY}" = true ] && [ "${GPU_ONLY}" = true ]; then
    echo "Error: --cpu-only and --gpu-only are mutually exclusive."
    exit 1
fi

# Resolve experiment selection
# --cpu-only and --gpu-only are shorthands
if [ "${CPU_ONLY}" = true ]; then
    RUN_MC=true
    RUN_IF=true
    RUN_YOLO=false
    EXPLICIT_SELECTION=true
elif [ "${GPU_ONLY}" = true ]; then
    RUN_MC=false
    RUN_IF=false
    RUN_YOLO=true
    EXPLICIT_SELECTION=true
fi

# If no explicit selection, run everything
if [ "${EXPLICIT_SELECTION}" = false ]; then
    RUN_MC=true
    RUN_IF=true
    RUN_YOLO=true
fi

# GPU detection
HAS_GPU=false
if command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null 2>&1; then
    HAS_GPU=true
fi

echo "========================================="
echo "Anytime ROS 2 — Artifact Evaluation"
echo "========================================="
echo ""
echo "Mode:          ${MODE}"
echo "Monte Carlo:   ${RUN_MC}"
echo "Interference:  ${RUN_IF}"
echo "YOLO:          ${RUN_YOLO}"
echo "Smoke test:    $([ "${NO_SMOKE}" = true ] && echo "skip" || echo "yes")"
echo "Prerequisites: $([ "${NO_PREREQS}" = true ] && echo "skip" || echo "yes")"
echo "Build:         $([ "${NO_BUILD}" = true ] && echo "skip" || echo "yes")"
echo "Figures:       $([ "${NO_FIGURES}" = true ] && echo "skip" || echo "yes")"
echo "GPU detected:  ${HAS_GPU}"
echo "Workspace:     ${WORKSPACE_DIR}"
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
if [ "${NO_SMOKE}" = true ]; then
    skip_phase "Smoke test" "--no-smoke-test flag set"
else
    run_phase "Smoke test" "${SCRIPT_DIR}/smoke_test.sh"
fi

# ─────────────────────────────────────────────
# Phase 2: Monte Carlo experiments
# ─────────────────────────────────────────────
if [ "${RUN_MC}" = true ]; then
    if [ "${MODE}" = "quick" ]; then
        run_phase "Monte Carlo experiments (quick)" "${WORKSPACE_DIR}/experiments/monte_carlo/run_quick.sh"
    else
        run_phase "Monte Carlo experiments (full)" "${WORKSPACE_DIR}/experiments/monte_carlo/run_monte_carlo_experiments.sh"
    fi
else
    skip_phase "Monte Carlo experiments" "not selected"
fi

# ─────────────────────────────────────────────
# Phase 3: Interference experiments
# ─────────────────────────────────────────────
if [ "${RUN_IF}" = true ]; then
    if [ "${MODE}" = "quick" ]; then
        run_phase "Interference experiments (quick)" "${WORKSPACE_DIR}/experiments/interference/run_quick.sh"
    else
        run_phase "Interference experiments (full)" "${WORKSPACE_DIR}/experiments/interference/run_interference_experiments.sh"
    fi
else
    skip_phase "Interference experiments" "not selected"
fi

# ─────────────────────────────────────────────
# Phase 4: YOLO experiments (GPU required)
# ─────────────────────────────────────────────
if [ "${RUN_YOLO}" = true ]; then
    if [ "${HAS_GPU}" = false ]; then
        skip_phase "YOLO experiments" "no GPU detected"
    elif [ "${MODE}" = "quick" ]; then
        skip_phase "YOLO experiments" "quick mode (YOLO requires --full)"
    else
        YOLO_DIR="${WORKSPACE_DIR}/experiments/yolo"

        echo ""
        echo "-----------------------------------------"
        echo "Phase: YOLO experiments (full pipeline)"
        echo "-----------------------------------------"

        # Check YOLO prerequisites (weights + images)
        yolo_prereqs_ok=true
        if [ "${NO_PREREQS}" = true ]; then
            echo "  Skipping prerequisite check (--no-prerequisites)"
        elif ! "${SCRIPT_DIR}/check_yolo_prerequisites.sh"; then
            echo "  FAILED: YOLO experiments (missing prerequisites)"
            failed=$((failed + 1))
            yolo_prereqs_ok=false
        fi

        # Only proceed if prerequisites passed
        if [ "${yolo_prereqs_ok}" = true ]; then

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

        fi  # end prerequisite/proceed check
    fi
else
    skip_phase "YOLO experiments" "not selected"
fi

# ─────────────────────────────────────────────
# Collect paper figures
# ─────────────────────────────────────────────
if [ "${NO_FIGURES}" = true ]; then
    skip_phase "Collect paper figures" "--no-figures flag set"
else
    echo ""
    echo "-----------------------------------------"
    echo "Collecting paper figures..."
    echo "-----------------------------------------"
    "${SCRIPT_DIR}/collect_figures.sh"
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
if [ "${RUN_MC}" = true ]; then
    echo "  Monte Carlo:    ${WORKSPACE_DIR}/experiments/monte_carlo/results/"
fi
if [ "${RUN_IF}" = true ]; then
    echo "  Interference:   ${WORKSPACE_DIR}/experiments/interference/results/"
fi
if [ "${RUN_YOLO}" = true ] && [ "${HAS_GPU}" = true ] && [ "${MODE}" = "full" ]; then
    echo "  YOLO:           ${WORKSPACE_DIR}/experiments/yolo/results/"
fi
echo ""
echo "  Paper figures:  ${WORKSPACE_DIR}/paper_figures/"
echo ""

if [ ${failed} -eq 0 ]; then
    echo "ALL PHASES PASSED"
    exit 0
else
    echo "SOME PHASES FAILED"
    exit 1
fi
