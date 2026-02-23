# 07 — Python CLI Tool (`anytime-eval`)

**Day:** 3 (Thu Feb 26)
**Priority:** CRITICAL
**Estimated time:** 3-4 hours
**Depends on:** 06 (Docker restructuring)

---

## Overview

Create a Python CLI tool that serves as the single entry point for artifact evaluation reviewers. Replaces the need to manually run Docker commands, build scripts, and experiment orchestration.

---

## Package Structure

```text
anytime_cli/
├── pyproject.toml
├── README.md
└── anytime_cli/
    ├── __init__.py
    ├── cli.py              # Main CLI entry point (click groups)
    ├── commands/
    │   ├── __init__.py
    │   ├── setup.py        # Environment detection + Docker build
    │   ├── build.py        # ROS2 workspace build + weight download
    │   ├── test.py         # Smoke test and full test suite
    │   ├── experiment.py   # Experiment orchestration
    │   ├── analyze.py      # Run analysis scripts
    │   └── run_all.py      # End-to-end pipeline
    ├── docker.py           # Docker build/run helpers
    └── utils.py            # Logging, environment detection, progress
```

---

## CLI Interface

```text
Usage: anytime-eval [OPTIONS] COMMAND [ARGS]...

  Anytime ROS 2 — Artifact Evaluation CLI

Options:
  --verbose / --quiet    Verbose output
  --dry-run              Show commands without executing
  --help                 Show this message and exit

Commands:
  setup       Detect environment and build Docker container
  build       Build ROS2 workspace and download dependencies
  test        Run smoke test or full test suite
  experiment  Run experiments (monte_carlo, interference, yolo, all)
  analyze     Generate analysis plots from collected traces
  run-all     End-to-end: setup -> build -> test -> experiment -> analyze
  status      Show current state (built? traces collected? results?)
```

---

## Subcommand Details

### `anytime-eval setup`

```text
Usage: anytime-eval setup [OPTIONS]

Options:
  --mode [gpu|cpu|jetson]  Hardware mode (default: auto-detect)
  --no-cache               Build Docker without cache
```

Steps:

1. Detect Docker installation
2. Detect NVIDIA toolkit (if mode=gpu)
3. Check disk space (need ~10GB for GPU, ~5GB for CPU)
4. Build Docker container using `docker compose build`
5. Print summary of detected environment

### `anytime-eval build`

```text
Usage: anytime-eval build [OPTIONS]

Options:
  --skip-weights    Skip YOLO weight download
  --skip-images     Skip test image download
  --clean           Clean build (remove build/ install/ first)
```

Steps:

1. Run `colcon build --symlink-install` inside container
2. Download YOLO weights (if not skipped and mode=gpu)
3. Download test images (if not skipped)
4. Verify build success

### `anytime-eval test`

```text
Usage: anytime-eval test [OPTIONS]

Options:
  --smoke    Quick validation only (<2 min)
  --full     Full test suite including GPU tests
```

Steps:

1. Run `colcon test` for selected packages
2. Run smoke test script
3. Report PASS/FAIL summary

### `anytime-eval experiment`

```text
Usage: anytime-eval experiment [OPTIONS] [NAME]

Arguments:
  NAME    Experiment to run: monte_carlo, interference, yolo, all

Options:
  --trials N     Number of trials per config (default: 3)
  --duration N   Duration per run in seconds (default: 10)
```

Steps:

1. Generate configs (if not already present)
2. Run experiment scripts inside container
3. Display progress with estimated time remaining
4. Collect traces

### `anytime-eval analyze`

```text
Usage: anytime-eval analyze [OPTIONS]

Options:
  --experiment NAME    Analyze specific experiment
  --output-dir PATH    Output directory for results
```

Steps:

1. Run Python analysis scripts on collected traces
2. Generate plots (PNG) and data files (CSV, JSON)
3. Print summary of results

### `anytime-eval run-all`

```text
Usage: anytime-eval run-all [OPTIONS]

Options:
  --mode [gpu|cpu|jetson]      Hardware mode
  --experiment [all|smoke]     Experiment scope
  --skip-setup                 Skip Docker build
```

Steps:

1. `setup` (unless `--skip-setup`)
2. `build`
3. `test --smoke`
4. `experiment` (selected experiments)
5. `analyze`
6. Print final summary with paths to results

---

## Key Implementation Details

### `anytime_cli/docker.py`

```python
import subprocess
import shutil

class DockerRunner:
    """Manages Docker container lifecycle for experiments."""

    def __init__(self, mode: str, dry_run: bool = False):
        self.mode = mode
        self.dry_run = dry_run
        self.compose_service = f"anytime-{mode}"

    def build(self, no_cache: bool = False) -> None:
        cmd = ["docker", "compose", "build", self.compose_service]
        if no_cache:
            cmd.append("--no-cache")
        self._run(cmd)

    def exec(self, command: str, workdir: str = "/home/vscode/workspace") -> str:
        cmd = [
            "docker", "compose", "run", "--rm",
            "-w", workdir,
            self.compose_service,
            "bash", "-c", command,
        ]
        return self._run(cmd)

    def _run(self, cmd: list[str]) -> str:
        if self.dry_run:
            print(f"[DRY RUN] {' '.join(cmd)}")
            return ""
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        return result.stdout
```

### `anytime_cli/utils.py`

```python
import shutil
import subprocess

def detect_environment() -> dict:
    """Detect available hardware and tools."""
    env = {
        "docker": shutil.which("docker") is not None,
        "nvidia_smi": shutil.which("nvidia-smi") is not None,
        "gpu_available": False,
        "disk_space_gb": 0,
    }

    if env["nvidia_smi"]:
        try:
            subprocess.run(["nvidia-smi"], capture_output=True, check=True)
            env["gpu_available"] = True
        except subprocess.CalledProcessError:
            pass

    # Check disk space
    import os
    stat = os.statvfs(".")
    env["disk_space_gb"] = (stat.f_frsize * stat.f_bavail) / (1024**3)

    return env

def auto_detect_mode() -> str:
    """Auto-detect the best mode based on hardware."""
    env = detect_environment()
    if env["gpu_available"]:
        return "gpu"
    return "cpu"
```

---

## `pyproject.toml`

```toml
[build-system]
requires = ["setuptools>=68.0", "wheel"]
build-backend = "setuptools.backends._legacy:_Backend"

[project]
name = "anytime-eval"
version = "1.0.0"
description = "Artifact evaluation CLI for Anytime ROS 2"
requires-python = ">=3.8"
license = {text = "Apache-2.0"}
dependencies = [
    "click>=8.0",
]

[project.scripts]
anytime-eval = "anytime_cli.cli:main"

[tool.setuptools.packages.find]
where = ["."]
```

---

## Verification

```bash
# Install CLI
pip install -e ./anytime_cli

# Test help
anytime-eval --help
anytime-eval setup --help
anytime-eval experiment --help

# Dry run
anytime-eval run-all --dry-run --mode cpu

# Full run (CPU-only)
anytime-eval run-all --mode cpu --experiment monte_carlo

# Full run (GPU)
anytime-eval run-all --mode gpu --experiment all
```
