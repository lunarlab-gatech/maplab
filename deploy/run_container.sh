DATA_DIR='/mnt/c/Users/camer/Documents/data'
REPO_DIR='/mnt/c/Users/camer/Documents/LunarLab/maplab'

docker run -it \
    --name="maplab" \
    --net="host" \
    --privileged \
    --gpus="all" \
    --device /dev/dri \
    --workdir="/home/$USER/maplab" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=/tmp/.Xauthority" \
    --env="XDG_RUNTIME_DIR=/tmp/runtime-$USER" \
    --env="USER_ID=$(id -u)" \
    --env="GROUP_ID=$(id -g)" \
    --volume="$REPO_DIR:/home/$USER/maplab" \
    --volume="$DATA_DIR:/home/$USER/data" \
    --volume="/home/$USER/.bash_aliases:/home/$USER/.bash_aliases" \
    --volume="/home/$USER/.ssh:/home/$USER/.ssh:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume /tmp/runtime-$USER:/tmp/runtime-$USER \
    maplab \
    /bin/bash