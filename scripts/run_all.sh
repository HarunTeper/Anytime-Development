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
# Cleanup:
#   --clean               Remove experiment outputs (traces, results, configs, figures, logs)
#   --clean-all           Remove everything --clean does, plus build artifacts and weights
#
# If none of --monte-carlo, --interference, --yolo, --cpu-only, --gpu-only
# are specified, all experiments are run.
#
# Duration estimates (on Jetson Orin NX):
#   --quick --cpu-only     ~5 minutes
#   --full  --cpu-only     ~15 minutes
#   --full  --gpu-only     ~1 hour
#   --full                 ~1.5 hours (all experiments)

set -e
export PYTHONUNBUFFERED=1

# Cleanup on interrupt
cleanup() {
    echo ""
    echo "Interrupted -- cleaning up..."
    if [ -n "${OUTPUT_DIR:-}" ] && [ -f "${TIMING_CSV:-}" ]; then
        echo "INTERRUPTED,Cleanup,INTERRUPTED,$(date +%s),$(date +%s),0" >> "${TIMING_CSV}"
    fi
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
DO_CLEAN=false
DO_CLEAN_ALL=false
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
        --clean) DO_CLEAN=true ;;
        --clean-all) DO_CLEAN_ALL=true ;;
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
            echo "Cleanup:"
            echo "  --clean               Remove experiment outputs (traces, results, configs,"
            echo "                        paper figures, evaluation logs)"
            echo "  --clean-all           Remove everything --clean does, plus build artifacts,"
            echo "                        YOLO weights, video frames, and __pycache__"
            echo ""
            echo "If no experiment flags are given, all experiments are run."
            echo "Flags can be combined: --monte-carlo --interference"
            echo ""
            echo "Duration estimates (on Jetson Orin NX):"
            echo "  --quick --cpu-only     ~5 minutes"
            echo "  --full  --cpu-only     ~15 minutes"
            echo "  --full  --gpu-only     ~1 hour"
            echo "  --full                 ~1.5 hours"
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
# Handle --clean / --clean-all (run and exit)
# ─────────────────────────────────────────────
if [ "${DO_CLEAN}" = true ] || [ "${DO_CLEAN_ALL}" = true ]; then
    echo "The following will be removed:"
    echo ""

    # Experiment outputs (always)
    echo "  Experiment outputs:"
    for exp in monte_carlo interference yolo; do
        for sub in traces results configs; do
            dir="${WORKSPACE_DIR}/experiments/${exp}/${sub}"
            if [ -d "${dir}" ]; then
                echo "    ${dir}/"
            fi
        done
    done

    if [ -d "${WORKSPACE_DIR}/paper_figures" ]; then
        echo "    ${WORKSPACE_DIR}/paper_figures/"
    fi
    if [ -d "${WORKSPACE_DIR}/eval_output" ]; then
        echo "    ${WORKSPACE_DIR}/eval_output/"
    fi

    if [ "${DO_CLEAN_ALL}" = true ]; then
        echo ""
        echo "  Build artifacts:"
        for sub in build install log; do
            dir="${PACKAGES_DIR}/${sub}"
            if [ -d "${dir}" ]; then
                echo "    ${dir}/"
            fi
        done

        echo ""
        echo "  YOLO weights and video frames:"
        weights_dir="${PACKAGES_DIR}/src/anytime_yolo/weights_32"
        if ls "${weights_dir}"/*.onnx 2>/dev/null | head -1 >/dev/null 2>&1 || [ -f "${weights_dir}/model.json" ]; then
            echo "    ${weights_dir}/*.onnx, *.engine, model.json, .gpu_fingerprint"
        fi
        images_dir="${PACKAGES_DIR}/src/video_publisher/images"
        if ls "${images_dir}"/image_*.jpg 2>/dev/null | head -1 >/dev/null 2>&1; then
            echo "    ${images_dir}/image_*.jpg"
        fi

        echo ""
        echo "  Python caches:"
        pycache_count=$(find "${WORKSPACE_DIR}" -type d -name "__pycache__" 2>/dev/null | wc -l)
        if [ "${pycache_count}" -gt 0 ]; then
            echo "    ${pycache_count} __pycache__ directories"
        fi
    fi

    echo ""
    read -r -p "Proceed? [y/N] " confirm
    if [ "${confirm}" != "y" ] && [ "${confirm}" != "Y" ]; then
        echo "Aborted."
        exit 0
    fi

    echo ""

    # Remove experiment outputs
    for exp in monte_carlo interference yolo; do
        for sub in traces results configs; do
            dir="${WORKSPACE_DIR}/experiments/${exp}/${sub}"
            if [ -d "${dir}" ]; then
                rm -rf "${dir}"
                echo "  Removed ${dir}/"
            fi
        done
    done
    if [ -d "${WORKSPACE_DIR}/paper_figures" ]; then
        rm -rf "${WORKSPACE_DIR}/paper_figures"
        echo "  Removed paper_figures/"
    fi
    if [ -d "${WORKSPACE_DIR}/eval_output" ]; then
        rm -rf "${WORKSPACE_DIR}/eval_output"
        echo "  Removed eval_output/"
    fi

    if [ "${DO_CLEAN_ALL}" = true ]; then
        for sub in build install log; do
            dir="${PACKAGES_DIR}/${sub}"
            if [ -d "${dir}" ]; then
                rm -rf "${dir}"
                echo "  Removed ${dir}/"
            fi
        done

        weights_dir="${PACKAGES_DIR}/src/anytime_yolo/weights_32"
        rm -f "${weights_dir}"/layer_*.onnx "${weights_dir}"/exit_*.onnx \
              "${weights_dir}"/nms.onnx "${weights_dir}"/model.json \
              "${weights_dir}"/combine_subheads_*.onnx "${weights_dir}"/subexit_*.onnx \
              "${weights_dir}"/layer_*.engine "${weights_dir}"/subexit_*.engine \
              "${weights_dir}"/combine_subheads_*.engine "${weights_dir}"/nms.engine \
              "${weights_dir}"/.gpu_fingerprint
        echo "  Removed YOLO weight and engine files"

        images_dir="${PACKAGES_DIR}/src/video_publisher/images"
        rm -f "${images_dir}"/image_*.jpg
        echo "  Removed video frames"

        find "${WORKSPACE_DIR}" -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
        echo "  Removed __pycache__ directories"
    fi

    echo ""
    echo "Cleanup complete."
    exit 0
fi

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

# ─────────────────────────────────────────────
# Output directory and timing setup
# ─────────────────────────────────────────────
RUN_TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="${WORKSPACE_DIR}/eval_output/run_${RUN_TIMESTAMP}"
mkdir -p "${OUTPUT_DIR}"

TIMING_CSV="${OUTPUT_DIR}/timing.csv"
echo "phase_num,phase_name,status,start_epoch,end_epoch,duration_seconds" > "${TIMING_CSV}"

OVERALL_START=$(date +%s)

declare -a PHASE_NAMES=()
declare -a PHASE_STATUSES=()
declare -a PHASE_DURATIONS=()
PHASE_COUNTER=0

echo "========================================="
echo "Anytime ROS 2 -- Artifact Evaluation"
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
echo "Output dir:    ${OUTPUT_DIR}"
echo ""

# ─────────────────────────────────────────────
# Capture hardware metadata
# ─────────────────────────────────────────────
{
    echo "Hardware metadata captured at $(date -u '+%Y-%m-%d %H:%M:%S UTC')"
    echo ""
    echo "--- OS / Kernel ---"
    uname -a
    echo ""
    echo "--- CPU ---"
    lscpu 2>/dev/null | head -20 || echo "(lscpu not available)"
    echo ""
    echo "--- Memory ---"
    head -3 /proc/meminfo 2>/dev/null || echo "(meminfo not available)"
    echo ""
    echo "--- GPU ---"
    if command -v nvidia-smi &>/dev/null; then
        nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv 2>/dev/null || echo "(nvidia-smi query failed)"
    else
        echo "(no NVIDIA GPU detected)"
    fi
} > "${OUTPUT_DIR}/hardware.txt" 2>&1

passed=0
failed=0
skipped=0

# ─────────────────────────────────────────────
# Helper functions
# ─────────────────────────────────────────────
format_duration() {
    local total_seconds=$1
    local hours=$((total_seconds / 3600))
    local minutes=$(( (total_seconds % 3600) / 60 ))
    local seconds=$((total_seconds % 60))
    printf "%02d:%02d:%02d" ${hours} ${minutes} ${seconds}
}

record_timing() {
    local phase_num="$1"
    local phase_name="$2"
    local status="$3"
    local start_epoch="$4"
    local end_epoch="$5"
    local duration=$((end_epoch - start_epoch))

    echo "${phase_num},${phase_name},${status},${start_epoch},${end_epoch},${duration}" >> "${TIMING_CSV}"

    PHASE_NAMES+=("${phase_name}")
    PHASE_STATUSES+=("${status}")
    PHASE_DURATIONS+=("${duration}")
}

run_phase() {
    local name="$1"
    local cmd="$2"
    PHASE_COUNTER=$((PHASE_COUNTER + 1))
    local phase_num
    phase_num=$(printf "%02d" ${PHASE_COUNTER})

    local log_name
    log_name=$(echo "${name}" | tr '[:upper:]' '[:lower:]' | tr ' ' '_' | tr -cd 'a-z0-9_')
    local log_file="${OUTPUT_DIR}/${phase_num}_${log_name}.log"

    echo ""
    echo "-----------------------------------------"
    echo "Phase: ${name}"
    echo "-----------------------------------------"
    echo "  Log: ${log_file}"

    local start_epoch
    start_epoch=$(date +%s)

    local exit_code=0
    set +e
    ( eval "${cmd}" ) 2>&1 | tee "${log_file}"
    exit_code=${PIPESTATUS[0]}
    set -e

    local end_epoch
    end_epoch=$(date +%s)
    local duration=$((end_epoch - start_epoch))

    if [ ${exit_code} -eq 0 ]; then
        echo "  PASSED: ${name} ($(format_duration ${duration}))"
        passed=$((passed + 1))
        record_timing "${phase_num}" "${name}" "PASSED" "${start_epoch}" "${end_epoch}"
    else
        echo "  FAILED: ${name} ($(format_duration ${duration}))"
        failed=$((failed + 1))
        record_timing "${phase_num}" "${name}" "FAILED" "${start_epoch}" "${end_epoch}"
    fi
}

skip_phase() {
    local name="$1"
    local reason="$2"
    PHASE_COUNTER=$((PHASE_COUNTER + 1))
    local phase_num
    phase_num=$(printf "%02d" ${PHASE_COUNTER})

    echo ""
    echo "-----------------------------------------"
    echo "Phase: ${name} -- SKIPPED (${reason})"
    echo "-----------------------------------------"
    skipped=$((skipped + 1))

    local now
    now=$(date +%s)
    record_timing "${phase_num}" "${name}" "SKIPPED" "${now}" "${now}"
}

# Run a YOLO sub-step with timing and log capture
run_yolo_step() {
    local step_label="$1"
    local description="$2"
    local cmd="$3"

    local log_file="${OUTPUT_DIR}/04_yolo_${step_label}.log"

    echo "  ${description}..."
    echo "    Log: ${log_file}"

    local start_epoch
    start_epoch=$(date +%s)

    local exit_code=0
    set +e
    ( eval "${cmd}" ) 2>&1 | tee "${log_file}"
    exit_code=${PIPESTATUS[0]}
    set -e

    local end_epoch
    end_epoch=$(date +%s)
    local duration=$((end_epoch - start_epoch))

    local status="PASSED"
    if [ ${exit_code} -ne 0 ]; then
        status="FAILED"
    fi
    record_timing "04" "YOLO ${description}" "${status}" "${start_epoch}" "${end_epoch}"

    echo "    Elapsed: $(format_duration ${duration})"

    return ${exit_code}
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
        PHASE_COUNTER=$((PHASE_COUNTER + 1))

        echo ""
        echo "-----------------------------------------"
        echo "Phase: YOLO experiments (full pipeline)"
        echo "-----------------------------------------"

        yolo_phase_start=$(date +%s)

        # Check YOLO prerequisites (weights + images)
        yolo_prereqs_ok=true
        if [ "${NO_PREREQS}" = true ]; then
            echo "  Skipping prerequisite check (--no-prerequisites)"
        else
            run_yolo_step "prerequisites" "Checking prerequisites" \
                "'${SCRIPT_DIR}/check_yolo_prerequisites.sh'" || {
                echo "  FAILED: YOLO experiments (missing prerequisites)"
                failed=$((failed + 1))
                yolo_prereqs_ok=false
            }
        fi

        # Only proceed if prerequisites passed
        if [ "${yolo_prereqs_ok}" = true ]; then

        cd "${PACKAGES_DIR}"
        source install/setup.bash

        # Pre-build TensorRT engines so warmup time is tracked separately
        run_yolo_step "engine_warmup" "Engine warmup (TensorRT compilation)" \
            "'${SCRIPT_DIR}/warmup_yolo_engines.sh'" || {
            echo "  WARNING: Engine warmup failed, experiments may still work"
        }

        yolo_failed=false

        run_yolo_step "step0_generate_baseline_configs" "Step 0: Generating baseline configs" \
            "python3 '${YOLO_DIR}/0_generate_baseline_configs.py'" || yolo_failed=true

        if [ "${yolo_failed}" = false ]; then
            run_yolo_step "step1_collect_baseline" "Step 1: Collecting baseline data" \
                "'${YOLO_DIR}/1_collect_baseline.sh'" || yolo_failed=true
        fi

        if [ "${yolo_failed}" = false ]; then
            run_yolo_step "step2a_analyze_quality" "Step 2a: Analyzing quality" \
                "python3 '${YOLO_DIR}/2a_analyze_quality.py'" || yolo_failed=true
        fi

        if [ "${yolo_failed}" = false ]; then
            run_yolo_step "step2b_analyze_blocks" "Step 2b: Analyzing block sizes" \
                "python3 '${YOLO_DIR}/2b_analyze_blocks.py'" || yolo_failed=true
        fi

        if [ "${yolo_failed}" = false ]; then
            run_yolo_step "step3_measure_throughput" "Step 3: Measuring throughput" \
                "'${YOLO_DIR}/3_measure_throughput.sh'" || yolo_failed=true
        fi

        if [ "${yolo_failed}" = false ]; then
            run_yolo_step "step4_analyze_throughput" "Step 4: Analyzing throughput" \
                "python3 '${YOLO_DIR}/4_analyze_throughput.py'" || yolo_failed=true
        fi

        if [ "${yolo_failed}" = false ]; then
            run_yolo_step "step5_generate_configs" "Step 5: Generating cancellation configs" \
                "python3 '${YOLO_DIR}/5_generate_configs.py'" || yolo_failed=true
        fi

        if [ "${yolo_failed}" = false ]; then
            run_yolo_step "step6_run_experiments" "Step 6: Running cancellation experiments" \
                "'${YOLO_DIR}/6_run_experiments.sh'" || yolo_failed=true
        fi

        if [ "${yolo_failed}" = false ]; then
            run_yolo_step "step7_analyze_cancellation" "Step 7: Analyzing cancellation" \
                "python3 '${YOLO_DIR}/7_analyze_cancellation.py'" || yolo_failed=true
        fi

        yolo_phase_end=$(date +%s)
        yolo_duration=$((yolo_phase_end - yolo_phase_start))

        if [ "${yolo_failed}" = false ]; then
            echo "  PASSED: YOLO experiments ($(format_duration ${yolo_duration}))"
            passed=$((passed + 1))
            record_timing "04" "YOLO experiments (total)" "PASSED" "${yolo_phase_start}" "${yolo_phase_end}"
        else
            echo "  FAILED: YOLO experiments ($(format_duration ${yolo_duration}))"
            failed=$((failed + 1))
            record_timing "04" "YOLO experiments (total)" "FAILED" "${yolo_phase_start}" "${yolo_phase_end}"
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
    run_phase "Collect paper figures" "${SCRIPT_DIR}/collect_figures.sh"
fi

# ─────────────────────────────────────────────
# Summary
# ─────────────────────────────────────────────
OVERALL_END=$(date +%s)
OVERALL_DURATION=$((OVERALL_END - OVERALL_START))
total=$((passed + failed + skipped))

{
    echo ""
    echo "========================================="
    echo "Evaluation Summary"
    echo "========================================="
    echo "  Passed:  ${passed}"
    echo "  Failed:  ${failed}"
    echo "  Skipped: ${skipped}"
    echo "  Total:   ${total}  (top-level phases)"
    echo ""
    echo "-----------------------------------------"
    echo "Timing Breakdown  (${#PHASE_NAMES[@]} steps)"
    echo "-----------------------------------------"
    printf "  %-45s %-10s %s\n" "Phase" "Status" "Duration"
    printf "  %-45s %-10s %s\n" "-----" "------" "--------"

    for i in "${!PHASE_NAMES[@]}"; do
        printf "  %-45s %-10s %s\n" \
            "${PHASE_NAMES[$i]}" \
            "${PHASE_STATUSES[$i]}" \
            "$(format_duration ${PHASE_DURATIONS[$i]})"
    done

    echo ""
    printf "  %-45s %-10s %s\n" "TOTAL" "" "$(format_duration ${OVERALL_DURATION})"
    echo ""
    echo "Result locations (relative to workspace root):"
    if [ "${RUN_MC}" = true ]; then
        echo "  Monte Carlo:    experiments/monte_carlo/results/"
    fi
    if [ "${RUN_IF}" = true ]; then
        echo "  Interference:   experiments/interference/results/"
    fi
    if [ "${RUN_YOLO}" = true ] && [ "${HAS_GPU}" = true ] && [ "${MODE}" = "full" ]; then
        echo "  YOLO:           experiments/yolo/results/"
    fi
    echo ""
    echo "  Paper figures:  paper_figures/"
    echo "  Run logs:       eval_output/run_${RUN_TIMESTAMP}/"
    echo "  Timing CSV:     eval_output/run_${RUN_TIMESTAMP}/timing.csv"
    echo "  Hardware info:  eval_output/run_${RUN_TIMESTAMP}/hardware.txt"
    echo ""

    if [ ${failed} -eq 0 ]; then
        echo "ALL PHASES PASSED"
    else
        echo "SOME PHASES FAILED"
    fi
} | tee "${OUTPUT_DIR}/summary.txt"

if [ ${failed} -eq 0 ]; then
    exit 0
else
    exit 1
fi
