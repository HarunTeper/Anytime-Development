FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0

# --- Configure environment ---

# Configure DEBIAN_FRONTEND to avoid warnings
ENV DEBIAN_FRONTEND=noninteractive

# set up work directory
WORKDIR /home/vscode/workspace

# --- TensorRT 10.3.0 is pre-installed in l4t-jetpack:r36.4.0 (with dev headers) ---

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
    libboost-python1.74-dev \
    sudo \
    bash-completion \
    vim \
    wget \
    iputils-ping \
    net-tools \
    zip && \
    rm -rf /var/lib/apt/lists/*

# --- Install ROS2 tracing (from source, not available as binary packages) ---

RUN /bin/bash -c '. /opt/ros/humble/setup.bash && \
    mkdir -p /home/vscode/ros2_tracing_ws/src && \
    git clone --depth 1 -b humble https://gitlab.com/ros-tracing/ros2_tracing.git /home/vscode/ros2_tracing_ws/src/ros2_tracing && \
    git clone --depth 1 -b humble https://github.com/ros-tracing/tracetools_analysis.git /home/vscode/ros2_tracing_ws/src/tracetools_analysis && \
    cd /home/vscode/ros2_tracing_ws && \
    colcon build --symlink-install --allow-overriding tracetools'

# --- Build cv_bridge from source (links against JetPack's OpenCV 4.8 instead of system 4.5) ---

RUN /bin/bash -c '. /opt/ros/humble/setup.bash && \
    mkdir -p /home/vscode/cv_bridge_ws/src && \
    git clone --depth 1 -b humble https://github.com/ros-perception/vision_opencv.git /home/vscode/cv_bridge_ws/src/vision_opencv && \
    cd /home/vscode/cv_bridge_ws && \
    colcon build --symlink-install --packages-select cv_bridge --allow-overriding cv_bridge'

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

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

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

# Configure ROS2 environment
RUN echo "" >> /home/$USERNAME/.bashrc && \
    echo "# ROS2 Environment Setup" >> /home/$USERNAME/.bashrc && \
    echo ". /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo ". /home/vscode/ros2_tracing_ws/install/setup.sh" >> /home/$USERNAME/.bashrc && \
    echo ". /home/vscode/cv_bridge_ws/install/setup.sh" >> /home/$USERNAME/.bashrc

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
