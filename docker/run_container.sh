# # Allow X11 GUI apps
# xhost +local:docker

# # Define directories
# DATA_DIR="/media/$(id -un)/T7/GT/SLAM"
# WS_DIR="/home/$(id -un)/Documents/ros_workspaces/maplab_ws/"

# docker run -it \
#     --name="maplab_ubuntu20_noetic_2" \
#     --shm-size=2gb \
#     --gpus="all" \
#     --network="host" \
#     --privileged \
#     --device /dev/dri \
#     --workdir="/home/$USER/maplab_ws" \
#     --env="DISPLAY=$DISPLAY" \
#     --env="QT_X11_NO_MITSHM=1" \
#     --env="XAUTHORITY=/tmp/.Xauthority" \
#     --env="XDG_RUNTIME_DIR=/tmp/runtime-$USER" \
#     --env="USER_ID=$(id -u)" \
#     --env="GROUP_ID=$(id -g)" \
#     --volume="$DATA_DIR:/home/$USER/data:rw" \
#     --volume="$WS_DIR:/home/$USER/maplab_ws:rw" \
#     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
#     --volume="/tmp/runtime-$USER:/tmp/runtime-$USER" \
#     --volume="$XAUTHORITY:/tmp/.Xauthority:ro" \
#     --volume="$HOME/.bash_aliases:/root/.bash_aliases:ro" \
#     --volume="$HOME/.ssh:/root/.ssh:ro" \
#     maplab_ubuntu20_noetic \
#     /bin/bash

#!/bin/bash

# Allow X11 GUI apps (Visualizers)
xhost +local:docker

# Define directories
DATA_DIR="$HOME/maplab_data"
WS_DIR="$HOME/maplab_ws"

# create if not
mkdir -p "$DATA_DIR"
mkdir -p "$WS_DIR"

docker run -it \
    --name="maplab_container" \
    --rm \
    --net=host \
    --privileged \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$DATA_DIR:/home/sidgirase/data:rw" \
    --volume="$WS_DIR:/home/sidgirase/maplab_ws:rw" \
    --workdir="/home/sidgirase/maplab_ws" \
    maplab_ubuntu20_noetic \
    /bin/bash
    