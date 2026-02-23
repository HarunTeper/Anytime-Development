# 06 — Docker Restructuring

**Day:** 3 (Thu Feb 26)
**Priority:** HIGH
**Estimated time:** 3-4 hours

---

## Overview

Extract shared content from the 3 Dockerfiles into a common base, then simplify each variant. Currently the 3 Dockerfiles share ~70% identical content (ROS2 install, LTTng, devtools, Python packages).

---

## Strategy: Shared Base + Variant Stages

### Current Structure

```text
.devcontainer/
├── linux/Dockerfile              # GPU: CUDA 12.5 + TensorRT + ROS2 + LTTng (~300 lines)
├── linux-no-hardware/Dockerfile  # CPU: ROS2 + LTTng (~250 lines)
└── jetson/Dockerfile             # ARM: l4t-jetpack + TensorRT + ROS2 + LTTng (~300 lines)
```

### Target Structure

```text
.devcontainer/
├── Dockerfile.base               # Shared: ROS2 + LTTng + devtools + Python (~200 lines)
├── linux/
│   ├── Dockerfile                # FROM base + CUDA 12.5 + TensorRT (~50 lines)
│   └── devcontainer.json
├── linux-no-hardware/
│   ├── Dockerfile                # FROM base only (~20 lines)
│   └── devcontainer.json
└── jetson/
    ├── Dockerfile                # Different base (l4t-jetpack) + TensorRT ARM64 (~80 lines)
    └── devcontainer.json
```

---

## New File: `.devcontainer/Dockerfile.base`

Contents extracted from the shared sections:

```dockerfile
ARG BASE_IMAGE=mcr.microsoft.com/devcontainers/base:jammy
FROM ${BASE_IMAGE}

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000

# ========== System packages ==========
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    clang \
    clang-format \
    clang-tidy \
    cppcheck \
    curl \
    git \
    locales \
    lsb-release \
    software-properties-common \
    wget \
    && rm -rf /var/lib/apt/lists/*

# ========== ROS 2 Humble (from source) ==========
# ... ros2.repos fetch, colcon build, install to /home/${USERNAME}/ros2_humble/

# ========== LTTng tracing stack ==========
RUN apt-get update && apt-get install -y --no-install-recommends \
    lttng-tools=2.13.* \
    liblttng-ust-dev=2.13.* \
    python3-babeltrace=1.5.* \
    python3-lttng \
    && rm -rf /var/lib/apt/lists/*

# ========== ROS 2 Tracing workspace ==========
# ... ros2_tracing, tracetools_analysis build

# ========== Python packages ==========
RUN pip3 install --no-cache-dir \
    pandas==2.0.3 \
    numpy==1.24.4 \
    matplotlib==3.7.5 \
    pyyaml==6.0.1 \
    opencv-python-headless==4.8.1.78

# ========== Tracing group setup ==========
RUN groupadd -f tracing && usermod -aG tracing ${USERNAME}

# ========== Workspace setup ==========
RUN mkdir -p /home/${USERNAME}/workspace
WORKDIR /home/${USERNAME}/workspace

# ========== Shell config ==========
COPY bashrc_additions.sh /tmp/
RUN cat /tmp/bashrc_additions.sh >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
```

---

## Modified Files

### `.devcontainer/linux/Dockerfile` (GPU variant)

```dockerfile
# Build base first
FROM anytime-base:latest AS base

# Or use multi-stage:
ARG BASE_IMAGE=mcr.microsoft.com/devcontainers/base:jammy
FROM ${BASE_IMAGE}

# ========== CUDA 12.5 ==========
# Install CUDA toolkit (pinned version)
RUN wget https://developer.download.nvidia.com/...cuda-repo-ubuntu2204_12.5.0-1_amd64.deb \
    && dpkg -i cuda-repo-*.deb \
    && apt-get update \
    && apt-get install -y cuda-toolkit-12-5=12.5.0-1 \
    && rm cuda-repo-*.deb

# ========== TensorRT 10.3.0 ==========
RUN apt-get install -y --no-install-recommends \
    libnvinfer-dev=10.3.0-1+cuda12.5 \
    libnvinfer-plugin-dev=10.3.0-1+cuda12.5

# Include base layers
# ... (shared ROS2, LTTng, Python from Dockerfile.base)

# ========== YOLO dependencies ==========
RUN apt-get install -y --no-install-recommends \
    nlohmann-json3-dev \
    libboost-python1.74-dev \
    dos2unix
```

### `.devcontainer/linux-no-hardware/Dockerfile` (CPU variant)

```dockerfile
# Simplest variant — just the base
FROM anytime-base:latest

# No additional layers needed
# All ROS2, LTTng, Python packages come from base
```

### `.devcontainer/jetson/Dockerfile` (ARM64 variant)

```dockerfile
# Jetson uses a different base image
FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0

# Cannot inherit from Dockerfile.base due to different base image
# But can use a shared script for common setup

# ========== TensorRT (ARM64) ==========
# Pre-installed in l4t-jetpack base

# ========== Shared setup (via script) ==========
COPY --from=base-builder /setup-scripts/ /tmp/setup-scripts/
RUN /tmp/setup-scripts/install-ros2.sh
RUN /tmp/setup-scripts/install-lttng.sh
RUN /tmp/setup-scripts/install-python-deps.sh
```

**Note:** The Jetson Dockerfile cannot directly inherit from the x86_64 base. Options:

1. Use shared shell scripts for common install steps
2. Keep Jetson Dockerfile mostly independent but with shared script includes
3. Accept some duplication for the Jetson variant (it has unique needs anyway)

**Recommendation:** Option 1 — shared install scripts in `.devcontainer/scripts/`

---

## New File: `.dockerignore`

```text
# Build artifacts
packages/build/
packages/install/
packages/log/

# Experiment output
experiments/*/traces/
experiments/*/results/

# Large files
*.pdf
*.bag

# IDE
.vscode/
*.code-workspace

# Git
.git/

# Python
__pycache__/
*.pyc
.pytest_cache/
```

---

## Additional Changes

### Version Pinning

Pin all `apt-get install` packages to specific versions where possible. This prevents subtle reproducibility issues when building months later.

### Layer Ordering Optimization

Order Dockerfile layers from least-changing to most-changing:

1. System packages (rarely change)
2. ROS2 from source (changes with ROS2 releases)
3. LTTng/tracing (rarely changes)
4. Python packages (changes with analysis updates)
5. Workspace setup (changes frequently)

---

## New File: `docker-compose.yml`

```yaml
version: "3.8"

services:
  anytime-gpu:
    build:
      context: .
      dockerfile: .devcontainer/linux/Dockerfile
    runtime: nvidia
    volumes:
      - .:/home/vscode/workspace
    environment:
      - NVIDIA_VISIBLE_DEVICES=all

  anytime-cpu:
    build:
      context: .
      dockerfile: .devcontainer/linux-no-hardware/Dockerfile
    volumes:
      - .:/home/vscode/workspace

  anytime-jetson:
    build:
      context: .
      dockerfile: .devcontainer/jetson/Dockerfile
    runtime: nvidia
    volumes:
      - .:/home/vscode/workspace
```

---

## Verification

```bash
# Build CPU variant
docker compose build anytime-cpu

# Build GPU variant
docker compose build anytime-gpu

# Test CPU variant
docker compose run anytime-cpu bash -c "cd packages && colcon build && colcon test"

# Test GPU variant
docker compose run anytime-gpu bash -c "cd packages && colcon build && colcon test"
```
