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

# --- Install ROS~2 Anytime dependencies ---

# install json library
RUN apt-get -y install nlohmann-json3-dev ros-humble-vision-msgs ros-humble-realsense2*

# --- Install Torch ---

# install libtorch
RUN mkdir /home/vscode/libraries && \
    cd /home/vscode/libraries && \
    wget https://download.pytorch.org/libtorch/cu121/libtorch-cxx11-abi-shared-with-deps-2.2.1%2Bcu121.zip && \
    unzip libtorch-cxx11-abi-shared-with-deps-2.2.1+cu121.zip && \
    rm libtorch-cxx11-abi-shared-with-deps-2.2.1+cu121.zip

# install CUDA

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin && \
    sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600 && \
    wget https://developer.download.nvidia.com/compute/cuda/12.1.0/local_installers/cuda-repo-ubuntu2204-12-1-local_12.1.0-530.30.02-1_amd64.deb && \
    sudo dpkg -i cuda-repo-ubuntu2204-12-1-local_12.1.0-530.30.02-1_amd64.deb && \
    sudo cp /var/cuda-repo-ubuntu2204-12-1-local/cuda-*-keyring.gpg /usr/share/keyrings/ && \
    sudo apt-get update && \
    sudo apt-get -y install cuda

ENV PATH=/usr/local/cuda-12.1/bin${PATH:+:${PATH}}
ENV LD_LIBRARY_PATH=/usr/local/cuda-12.1/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
ENV CUDACXX=/usr/local/cuda-12.1/bin/nvcc
ENV CUDA_NVRTC_LIB=/usr/local/cuda-12.1/lib64/libnvrtc.so

RUN apt-get install -y python-is-python3

# --- Set up user ---

# set user
USER vscode

# source ROS2 setup file
RUN echo '. /opt/ros/humble/setup.sh' >> /home/vscode/.bashrc

# configure environment variables
ENV DISPLAY=unix:1
ENV ROS_LOCALHOST_ONLY=0
ENV ROS_DOMAIN_ID=0

# --- Set up entrypoint ---

SHELL ["/bin/bash", "-c"]