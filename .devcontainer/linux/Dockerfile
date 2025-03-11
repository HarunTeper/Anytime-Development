ARG VARIANT
FROM mcr.microsoft.com/devcontainers/base:${VARIANT}

# --- Configure environment ---

# Configure DEBIAN_FRONTEND to avoid warnings
ENV DEBIAN_FRONTEND=noninteractive

# --- Install ROS2 ---
ARG INSTALL_ROS2=true
RUN if [ "$INSTALL_ROS2" = "true" ]; then \
    apt-get update && apt-get install -y locales && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    apt-get update && \
    apt-get install -y software-properties-common curl git && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y ros-humble-desktop libgeographic-dev python3-pip python3-colcon-common-extensions ros-dev-tools; \
    fi

# --- Install ROS2 tracing ---
ARG INSTALL_ROS2_TRACING=true
RUN if [ "$INSTALL_ROS2_TRACING" = "true" ]; then \
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
    . /opt/ros/humble/setup.sh &&\
    colcon build --symlink-install --allow-overriding tracetools && \
    pip install bokeh jupyter; \
    fi

# --- Install ROS 2 Extensions ---
RUN if [ "$INSTALL_ROS2" = "true" ]; then \
    apt-get update && \
    apt-get install -y ros-humble-rmw-cyclonedds-cpp; \
    fi

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

RUN wget https://developer.nvidia.com/downloads/compute/machine-learning/tensorrt/10.3.0/local_repo/nv-tensorrt-local-repo-ubuntu2204-10.3.0-cuda-12.5_1.0-1_amd64.deb && \
    dpkg -i nv-tensorrt-local-repo-ubuntu2204-10.3.0-cuda-12.5_1.0-1_amd64.deb && \
    cp /var/nv-tensorrt-local-repo-ubuntu2204-10.3.0-cuda-12.5/*-keyring.gpg /usr/share/keyrings/ && \
    apt-get update && \
    apt-get install -y tensorrt && \
    rm nv-tensorrt-local-repo-ubuntu2204-10.3.0-cuda-12.5_1.0-1_amd64.deb

# --- Anytime Dependencies ---
RUN if [ "$INSTALL_ROS2" = "true" ]; then \
    apt-get install -y ros-humble-ament-cmake-google-benchmark && \
    pip install pandas "numpy<2.0"; \
    fi

# --- Install DevTools ---
RUN apt-get -y install clangd clang-format clang-tidy cppcheck
RUN pip install pep8 pylint

# --- Set up user ---
USER vscode
WORKDIR /home/vscode/workspace

# source ROS2 setup file and workspace
RUN if [ "$INSTALL_ROS2" = "true" ]; then \
    echo '. /opt/ros/humble/setup.sh' >> /home/vscode/.bashrc && \
    echo "if [ -f ~/workspace/packages/install/setup.bash ]; then source ~/workspace/packages/install/setup.bash; fi" >> /home/vscode/.bashrc && \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
    CYCLONEDDS_URI=/home/vscode/workspace/cyclone_dds_config.xml; \
    fi

# source ROS2 tracing setup file
RUN if [ "$INSTALL_ROS2_TRACING" = "true" ]; then \
    echo '. /home/vscode/ros2_tracing_ws/install/setup.sh' >> /home/vscode/.bashrc; \
    fi

# # --- Install Realsense ---
# RUN cd /home/vscode && \
#     git clone https://github.com/IntelRealSense/librealsense.git && \
#     cd librealsense && \
#     mkdir build && cd build && \
#     cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true -DFORCE_RSUSB_BACKEND=true -DBUILD_WITH_CUDA=true && \
#     make -j$(($(nproc)-1)) && \
#     sudo make install

# RUN cd /home/vscode && \
#     mkdir -p ros2_realsense_ws/src && \
#     cd ros2_realsense_ws/src && \
#     git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master && \
#     cd .. && \
#     . /opt/ros/humble/setup.sh && \
#     sudo apt-get install python3-rosdep -y && \
#     sudo rosdep init && \
#     rosdep update && \
#     rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y && \
#     colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
#     echo '. /home/vscode/ros2_realsense_ws/install/setup.sh' >> /home/vscode/.bashrc

# configure environment variables
ARG DISPLAY
ARG ROS_LOCALHOST_ONLY
ARG ROS_DOMAIN_ID
ENV DISPLAY=${DISPLAY}
ENV ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# --- Set up entrypoint ---

SHELL ["/bin/bash", "-c"]