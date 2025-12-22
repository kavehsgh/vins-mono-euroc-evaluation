#!/bin/bash
echo "Starting VINS-Mono for Linux..."

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

# 1. Allow X11 connections (required for GUI)
xhost +local:docker

# 2. Build the image
echo "Building Docker image..."
docker build -t vins_mono_node docker/

# 3. Run the container
# We use 'docker run' instead of compose here to easily handle the X11 socket mounting
echo "Running container..."
docker run -it --rm --net=host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$SCRIPT_DIR/data":/root/catkin_ws/data \
    -v "$SCRIPT_DIR/config":/root/catkin_ws/config_overrides \
    -v "$SCRIPT_DIR/scripts":/root/catkin_ws/scripts \
    --name vins_mono_container \
    vins_mono_node \
    /bin/bash -c "source /entrypoint.sh && bash"
