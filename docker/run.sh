# Allow X11 GUI apps
xhost +local:docker

# Define directories
DATA_DIR='/media/nisemono/T7/GT/SLAM'
REPO_DIR='/root/maplab_ws'   # adjust this if you want to mount your source repo

docker run -it \
    --name="maplab_2" \
    --gpus="all" \
    --network="host" \
    --privileged \
    --device /dev/dri \
    --workdir="/root/maplab_ws" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=/tmp/.Xauthority" \
    --env="XDG_RUNTIME_DIR=/tmp/runtime-$USER" \
    --env="USER_ID=$(id -u)" \
    --env="GROUP_ID=$(id -g)" \
    --volume="$DATA_DIR:/data:rw" \
    --volume="$REPO_DIR:/root/maplab_ws:rw" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/tmp/runtime-$USER:/tmp/runtime-$USER" \
    --volume="$XAUTHORITY:/tmp/.Xauthority:ro" \
    --volume="$HOME/.bash_aliases:/root/.bash_aliases:ro" \
    --volume="$HOME/.ssh:/root/.ssh:ro" \
    maplab_ubuntu20_noetic \
    /bin/bash
