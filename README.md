# Maplab



Our fork of the [Maplab](https://github.com/ethz-asl/maplab) repository for evaluation as a baseline running on the HERCULES dataset.

## Install

### Docker Setup

Make sure to install:
- [Docker](https://docs.docker.com/engine/install/ubuntu/)

Then, clone this repository into a desired location on your computer.

After that, navigate to the `docker` directory. Log in to the user that you want the docker file to create in the container. Then, edit the `DOCKERFILE` to update these lines:
- `ARG USERNAME=`: Your username
- `ARG USER_UID=`: Output of `echo $UID`
- `ARG USER_GID=`: Output of `id -g`

Edit the `enter_container.sh` script with the following paths:
- `DATA_DIR=`: The directory where the HERCULES dataset is located
- `REPO_DIR=`: The directory of this repository

Now, run the following commands:
```
build_container.sh
run_container.sh
```

The rest of this README **assumes that you are inside the Docker container**. For easier debugging and use, its highly recommended to install the [VSCode Docker extension](https://code.visualstudio.com/docs/containers/overview), which allows you to start/stop the container and additionally attach VSCode to the container by right-clicking on the container and selecting `Attach Visual Studio Code`. If that isn't possible, you can re-enter the container running the following command:
```
enter_container.sh
```

### Maplab Install

Maplab will be built after setting up docker.

## Examples

### EuRoC Dataset

First, make changes to the following variables (to update data and user directory names):
```
maplab_server_merged_map_folder (applications/maplab-server-node/cfg/maplab_server_ros_params.yaml)
euroc_bags_folder (applications/maplab-server-node/launch/euroc/euroc-maplab-server-robots.launch)
euroc_root_map_directory (applications/maplab-server-node/launch/euroc/euroc-maplab-server-robots.launch)
```

Then run the following command to run maplab:
```
tmuxp load applications/maplab-server-node/launch/euroc/euroc_tmuxp_launch.yaml
```

After that, run the maplab-console with the following:
```
applications/maplab-server-node/launch/euroc/euroc_vis_map.yaml
```

The maplab-console will specify where the output trajectories were saved.