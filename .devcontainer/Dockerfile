ARG VARIANT=ubuntu-22.04
FROM mcr.microsoft.com/vscode/devcontainers/base:0-${VARIANT}

# --- Configure environment ---

# Configure DEBIAN_FRONTEND to avoid warnings
ENV DEBIAN_FRONTEND=noninteractive

# set up work directory
WORKDIR /home/vscode/workspace

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

# install ROS2
RUN apt-get update && \
    apt-get install -y ros-humble-desktop

# install ROS2 development tools
RUN apt-get -y install python3-pip python3-colcon-common-extensions

# install json library
RUN apt-get update && apt-get -y install nlohmann-json3-dev ros-humble-vision-msgs ros-humble-realsense2*

# --- Install ROS2 Tracing ---

RUN apt-get install -y lttng-tools liblttng-ust-dev && \
    apt-get install -y python3-babeltrace python3-lttng && \
    apt-get install -y lttng-modules-dkms

# set up user for tracing
RUN groupadd -f -r tracing && \
    usermod -a -G tracing vscode

RUN mkdir -p /home/vscode/ros2_tracing_ws/src/ros2_tracing && \
    git clone -b humble https://gitlab.com/ros-tracing/ros2_tracing.git /home/vscode/ros2_tracing_ws/src/ros2_tracing

# build ROS2 tracing
RUN cd /home/vscode/ros2_tracing_ws &&\
    . /opt/ros/humble/setup.sh &&\
    colcon build --symlink-install --allow-overriding tracetools

# --- Install CUDA and LibTorch ---

# install libtorch
RUN mkdir /home/vscode/libraries && \
    cd /home/vscode/libraries && \
    wget https://download.pytorch.org/libtorch/cu121/libtorch-cxx11-abi-shared-with-deps-2.2.1%2Bcu121.zip && \
    unzip libtorch-cxx11-abi-shared-with-deps-2.2.1+cu121.zip && \
    rm libtorch-cxx11-abi-shared-with-deps-2.2.1+cu121.zip

# install CUDA
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
RUN mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
RUN wget https://developer.download.nvidia.com/compute/cuda/12.5.0/local_installers/cuda-repo-ubuntu2204-12-5-local_12.5.0-555.42.02-1_amd64.deb
RUN dpkg -i cuda-repo-ubuntu2204-12-5-local_12.5.0-555.42.02-1_amd64.deb
RUN cp /var/cuda-repo-ubuntu2204-12-5-local/cuda-*-keyring.gpg /usr/share/keyrings/
RUN apt-get update
RUN apt-get -y install cuda-toolkit-12-5

ENV PATH=/usr/local/cuda-12.5/bin${PATH:+:${PATH}}
ENV LD_LIBRARY_PATH=/usr/local/cuda-12.5/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
ENV CUDACXX=/usr/local/cuda-12.5/bin/nvcc
ENV CUDA_NVRTC_LIB=/usr/local/cuda-12.5/lib64/libnvrtc.so

RUN apt-get install -y python-is-python3

# --- Anytime Dependencies ---

RUN apt-get install -y ros-humble-ament-cmake-google-benchmark

# --- Install DevTools ---

RUN apt-get -y install clangd clang-format clang-tidy cppcheck
RUN pip install pep8 pylint

# --- Set up user ---

# set user
USER vscode

# source ROS2 setup file
RUN echo '. /opt/ros/humble/setup.sh' >> /home/vscode/.bashrc

# set up ROS2 tracing
RUN echo '. /home/vscode/ros2_tracing_ws/install/setup.sh' >> /home/vscode/.bashrc

# configure environment variables
ENV DISPLAY=unix:1
ENV ROS_LOCALHOST_ONLY=0
ENV ROS_DOMAIN_ID=0

# --- Set up entrypoint ---

SHELL ["/bin/bash", "-c"]