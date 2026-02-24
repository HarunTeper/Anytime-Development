FROM mcr.microsoft.com/devcontainers/base:jammy

# --- Configure environment ---

# Configure DEBIAN_FRONTEND to avoid warnings
ENV DEBIAN_FRONTEND=noninteractive

# set up work directory
WORKDIR /home/vscode/workspace

# --- Add NVIDIA network repository ---

RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && \
    rm cuda-keyring_1.1-1_all.deb

# --- Install CUDA Toolkit 12.5 and TensorRT 10.3 (pinned to cuda12.5) ---

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        cuda-toolkit-12-5 \
        tensorrt-dev=10.3.0.26-1+cuda12.5 \
        tensorrt-libs=10.3.0.26-1+cuda12.5 \
        libnvinfer-dev=10.3.0.26-1+cuda12.5 \
        libnvinfer-dispatch-dev=10.3.0.26-1+cuda12.5 \
        libnvinfer-dispatch10=10.3.0.26-1+cuda12.5 \
        libnvinfer-headers-dev=10.3.0.26-1+cuda12.5 \
        libnvinfer-headers-plugin-dev=10.3.0.26-1+cuda12.5 \
        libnvinfer-lean-dev=10.3.0.26-1+cuda12.5 \
        libnvinfer-lean10=10.3.0.26-1+cuda12.5 \
        libnvinfer-plugin-dev=10.3.0.26-1+cuda12.5 \
        libnvinfer-plugin10=10.3.0.26-1+cuda12.5 \
        libnvinfer-vc-plugin-dev=10.3.0.26-1+cuda12.5 \
        libnvinfer-vc-plugin10=10.3.0.26-1+cuda12.5 \
        libnvinfer10=10.3.0.26-1+cuda12.5 \
        libnvonnxparsers-dev=10.3.0.26-1+cuda12.5 \
        libnvonnxparsers10=10.3.0.26-1+cuda12.5 && \
    rm -rf /var/lib/apt/lists/*

# --- Configure CUDA paths ---

ENV PATH="/usr/local/cuda-12.5/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda-12.5/lib64:${LD_LIBRARY_PATH}"

# --- Install locales and add ROS2 repository ---

RUN apt-get update && apt-get install -y \
    locales \
    software-properties-common \
    curl \
    git && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    rm -rf /var/lib/apt/lists/*

# set locale environment variables
ENV LANG=en_US.UTF-8

# --- Install ROS2 (binary), tracing, dev tools, and YOLO dependencies ---

RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-vision-msgs \
    ros-humble-rmw-cyclonedds-cpp \
    lttng-tools \
    liblttng-ust-dev \
    python3-babeltrace \
    python3-lttng \
    lttng-modules-dkms \
    python3-pip \
    python3-rosdep \
    ros-dev-tools \
    clangd \
    clang \
    clang-format \
    clang-tidy \
    cppcheck \
    build-essential \
    cmake \
    libc++-dev \
    libstdc++-12-dev \
    libc++abi-dev \
    nlohmann-json3-dev \
    dos2unix \
    libboost-python1.74-dev && \
    rm -rf /var/lib/apt/lists/*

# --- Install ROS2 tracing (from source, not available as binary packages) ---

RUN /bin/bash -c '. /opt/ros/humble/setup.bash && \
    mkdir -p /home/vscode/ros2_tracing_ws/src && \
    git clone --depth 1 -b humble https://gitlab.com/ros-tracing/ros2_tracing.git /home/vscode/ros2_tracing_ws/src/ros2_tracing && \
    git clone --depth 1 -b humble https://github.com/ros-tracing/tracetools_analysis.git /home/vscode/ros2_tracing_ws/src/tracetools_analysis && \
    cd /home/vscode/ros2_tracing_ws && \
    colcon build --symlink-install --allow-overriding tracetools'

# --- Set up tracing group ---

RUN groupadd -f -r tracing && \
    usermod -a -G tracing vscode

# --- Python dependencies ---

RUN pip install \
    "numpy<2.0" \
    pandas \
    matplotlib \
    bokeh \
    jupyter \
    pep8 \
    pylint \
    opencv-python

# --- Set up user ---

USER vscode

# source ROS2 setup file and tracing workspace
RUN echo '. /opt/ros/humble/setup.bash' >> /home/vscode/.bashrc && \
    echo '. /home/vscode/ros2_tracing_ws/install/setup.sh' >> /home/vscode/.bashrc

# configure environment variables
ARG DISPLAY
ARG ROS_LOCALHOST_ONLY
ARG ROS_DOMAIN_ID
ENV DISPLAY=${DISPLAY}
ENV ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# --- Set up entrypoint ---

SHELL ["/bin/bash", "-c"]
