FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0

# --- Configure environment ---

# Configure DEBIAN_FRONTEND to avoid warnings
ENV DEBIAN_FRONTEND=noninteractive

# set up work directory
WORKDIR /home/vscode/workspace

# --- Install TensorRT ---

RUN wget https://developer.nvidia.com/downloads/compute/machine-learning/tensorrt/10.3.0/local_repo/nv-tensorrt-local-repo-ubuntu2204-10.3.0-cuda-12.5_1.0-1_arm64.deb && \
    dpkg -i nv-tensorrt-local-repo-ubuntu2204-10.3.0-cuda-12.5_1.0-1_arm64.deb && \
    cp /var/nv-tensorrt-local-repo-ubuntu2204-10.3.0-cuda-12.5/*-keyring.gpg /usr/share/keyrings/ && \
    apt-get update && \
    apt-get install -y tensorrt && \
    rm nv-tensorrt-local-repo-ubuntu2204-10.3.0-cuda-12.5_1.0-1_arm64.deb

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

# # --- Install Realsense ---

# RUN apt-get -y install libxinerama-dev libxcursor-dev

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
#     . /home/vscode/ros2_humble/install/setup.sh && \
#     sudo apt-get install python3-rosdep -y && \
#     sudo rosdep init && \
#     rosdep update && \
#     rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y && \
#     colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release

# --- Anytime Dependencies ---

RUN pip install pandas "numpy<2.0" && \
    pip install --upgrade "numpy<2.0" matplotlib pandas

# --- Yolo Dependencies ---

RUN apt-get -y install nlohmann-json3-dev dos2unix libboost-python1.74-dev
RUN pip install opencv-python cv_bridge

# --- Set up user ---

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Install bash completion and other helpful tools
RUN apt-get update && apt-get install -y \
    sudo \
    bash-completion \
    vim \
    wget \
    iputils-ping \
    net-tools \
    zip

# Create the user with proper shell and environment
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    # Add sudo support
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    # Copy skeleton files for bash configuration
    && cp -r /etc/skel/. /home/$USERNAME/ \
    # Create proper .bashrc if it doesn't exist or is empty
    && if [ ! -s /home/$USERNAME/.bashrc ]; then \
    echo '# ~/.bashrc: executed by bash(1) for non-login shells.' > /home/$USERNAME/.bashrc && \
    echo 'case $- in' >> /home/$USERNAME/.bashrc && \
    echo '    *i*) ;;' >> /home/$USERNAME/.bashrc && \
    echo '      *) return;;' >> /home/$USERNAME/.bashrc && \
    echo 'esac' >> /home/$USERNAME/.bashrc && \
    echo 'export PATH=$HOME/.local/bin:$PATH' >> /home/$USERNAME/.bashrc && \
    echo 'if [ -f /etc/bash_completion ] && ! shopt -oq posix; then' >> /home/$USERNAME/.bashrc && \
    echo '    . /etc/bash_completion' >> /home/$USERNAME/.bashrc && \
    echo 'fi' >> /home/$USERNAME/.bashrc; \
    fi \
    # Set proper ownership
    && chown -R $USERNAME:$USERNAME /home/$USERNAME

# Set the default user
USER $USERNAME

# Add user to necessary groups
RUN sudo usermod -a -G video $USERNAME && \
    sudo groupadd -f -r tracing && \
    sudo usermod -a -G tracing $USERNAME

# Configure ROS2 environment - use separate command for each line
RUN echo "" >> /home/$USERNAME/.bashrc && \
    echo "# ROS2 Environment Setup" >> /home/$USERNAME/.bashrc && \
    echo ". /home/vscode/ros2_humble/install/setup.sh" >> /home/$USERNAME/.bashrc && \
    echo ". /home/vscode/ros2_tracing_ws/install/setup.sh" >> /home/$USERNAME/.bashrc
#    && echo ". /home/vscode/ros2_realsense_ws/install/setup.sh" >> /home/$USERNAME/.bashrc

# configure environment variables
ARG DISPLAY
ARG ROS_LOCALHOST_ONLY
ARG ROS_DOMAIN_ID
ENV DISPLAY=${DISPLAY}
ENV ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# --- Set up entrypoint ---

# Set default shell to /bin/bash
SHELL ["/bin/bash", "-c"]
