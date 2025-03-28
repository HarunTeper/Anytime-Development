# Anytime-Development

## Setup

Install [Docker Engine](https://docs.docker.com/engine/install/)

Install [Visual Studio Code](https://code.visualstudio.com/)

## GPU Setup

Follow the instructions for the [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

## Start the Container with Visual Studio Code

In Visual Studio Code, install the **Docker** and **Dev Containers** extensions.

Build and open the container.

## Start the Container with Docker

Run the following command in a terminal that is in the current folder's directory:

> docker build -t ros2-anytime .devcontainer/

Start the container using the following command:

> docker run --gpus all --network host -it -v .:/home/vscode/workspace ros2-anytime

## Running experiments

> tmux new-session -s evaluation-session

> cd /home/vscode/workspace
docker build \
  -t ros2-anytime-jetson:latest \
  -f .devcontainer/jetson/Dockerfile \
  --build-arg ROS_DOMAIN_ID=12 \
  --build-arg ROS_LOCALHOST_ONLY=1 \
  --build-arg DISPLAY=$DISPLAY \
  .devcontainer

>docker run \
  --name ros2-anytime-jetson \
  --network=host \
  --privileged \
  --gpus=all \
  --runtime=nvidia \
  --env=NVIDIA_VISIBLE_DEVICES=all \
  --env=ROS_DOMAIN_ID=12 \
  --env=ROS_LOCALHOST_ONLY=1 \
  --env=DISPLAY=$DISPLAY \
  --env=SHELL=/bin/bash \
  -v ${PWD}:/home/vscode/workspace \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -u vscode \
  -w /home/vscode/workspace \
  ros2-anytime-jetson:latest \
  /bin/bash -c "chmod +x /home/vscode/workspace/evaluation_monte_carlo.sh && /home/vscode/workspace/evaluation_monte_carlo.sh"