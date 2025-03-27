FROM mcr.microsoft.com/devcontainers/base:jammy

# --- Configure environment ---

# Configure DEBIAN_FRONTEND to avoid warnings
ENV DEBIAN_FRONTEND=noninteractive

# set up work directory
WORKDIR /home/vscode/workspace

# --- Install CUDA ---

RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin && \
    mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600 && \
    wget https://developer.download.nvidia.com/compute/cuda/12.5.1/local_installers/cuda-repo-ubuntu2204-12-5-local_12.5.1-555.42.06-1_amd64.deb && \
    dpkg -i cuda-repo-ubuntu2204-12-5-local_12.5.1-555.42.06-1_amd64.deb && \
    cp /var/cuda-repo-ubuntu2204-12-5-local/cuda-*-keyring.gpg /usr/share/keyrings/ && \
    apt-get update && \
    apt-get -y install cuda-toolkit-12-5

# --- Configure CUDA paths ---

ENV PATH="/usr/local/cuda-12.5/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda-12.5/lib64:${LD_LIBRARY_PATH}"

# --- Install TensorRT ---

RUN wget https://developer.nvidia.com/downloads/compute/machine-learning/tensorrt/10.3.0/local_repo/nv-tensorrt-local-repo-ubuntu2204-10.3.0-cuda-12.5_1.0-1_amd64.deb && \
    dpkg -i nv-tensorrt-local-repo-ubuntu2204-10.3.0-cuda-12.5_1.0-1_amd64.deb && \
    cp /var/nv-tensorrt-local-repo-ubuntu2204-10.3.0-cuda-12.5/*-keyring.gpg /usr/share/keyrings/ && \
    apt-get update && \
    apt-get install -y tensorrt && \
    rm nv-tensorrt-local-repo-ubuntu2204-10.3.0-cuda-12.5_1.0-1_amd64.deb

# --- Install ROS2 ---

# install locales for UTF-8 support
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# set locale environment variables
ENV LANG=en_US.UTF-8

# add ROS2 repository
RUN apt-get update && \
    apt-get install -y software-properties-common curl git && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# install ROS2 dependencies
RUN apt-get update && apt-get install -y \
    python3-flake8-docstrings \
    python3-pip \
    python3-pytest-cov \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-vcstool \
    python3-rosdep \
    ros-dev-tools

# install ROS2
RUN mkdir -p /home/vscode/ros2_humble/src && \
    cd /home/vscode/ros2_humble && \
    vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src && \
    apt-get update && \
    sudo rosdep init && \
    rosdep update && \
    apt-get update && \
    apt-get -y upgrade && \
    cd /home/vscode/ros2_humble && \
    rosdep install --rosdistro humble --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers" && \
    cd /home/vscode/ros2_humble && \
    colcon build --symlink-install

# --- Install ROS2 tracing ---

RUN . /home/vscode/ros2_humble/install/setup.sh && \
    apt-get update && \
    apt-get install -y lttng-tools liblttng-ust-dev && \
    apt-get install -y python3-babeltrace python3-lttng && \
    apt-get install -y lttng-modules-dkms && \
    groupadd -f -r tracing && \
    usermod -a -G tracing vscode && \
    mkdir -p /home/vscode/ros2_tracing_ws/src/ros2_tracing && \
    git clone -b humble https://gitlab.com/ros-tracing/ros2_tracing.git /home/vscode/ros2_tracing_ws/src/ros2_tracing && \
    mkdir -p /home/vscode/ros2_tracing_ws/src/tracetools_analysis && \
    git clone -b humble https://github.com/ros-tracing/tracetools_analysis.git /home/vscode/ros2_tracing_ws/src/tracetools_analysis && \
    cd /home/vscode/ros2_tracing_ws &&\
    colcon build --symlink-install --allow-overriding tracetools && \
    pip install bokeh jupyter

# --- Install DevTools ---

RUN apt-get update && \
    apt-get -y install \
    python3-pip \
    clangd \
    clang \
    clang-format \
    clang-tidy \
    cppcheck \
    build-essential \
    cmake \
    libc++-dev \
    libstdc++-12-dev \
    libc++abi-dev && \
    pip install pep8 pylint

# --- Anytime Dependencies ---

RUN pip install pandas "numpy<2.0" && \
    pip install --upgrade "numpy<2.0" matplotlib pandas

# --- Yolo Dependencies ---

RUN apt-get -y install nlohmann-json3-dev dos2unix libboost-python1.74-dev
RUN pip install opencv-python cv_bridge

# --- Set up user ---

USER vscode

# source ROS2 setup file and workspace
RUN echo '. /home/vscode/ros2_humble/install/setup.sh' >> /home/vscode/.bashrc && \
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