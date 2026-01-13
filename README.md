# Maplab

Our fork of the [Maplab](https://github.com/ethz-asl/maplab) repository for evaluation as a baseline running on the HERCULES dataset.

## Install

### Docker Setup

Make sure to install:
- [Docker](https://docs.docker.com/engine/install/ubuntu/)

Then, clone this repository into a `src` folder in a ros workspace on your computer with the following command:
```
git clone git@github.com:lunarlab-gatech/maplab.git --recursive -b master
```

After that, navigate to the `docker` directory. Log in to the user that you want the docker file to create in the container. Then, edit the `enter_container.sh` script with the following paths:
- `DATA_DIR=`: The directory where the datasets are located
- `WS_DIR=`: The directory of the ROS workspace

Now, run the following commands:
```
build_container.sh
run_container.sh
```

The rest of this README **assumes that you are inside the Docker container**. For easier debugging and use, its highly recommended to install the [VSCode Docker extension](https://code.visualstudio.com/docs/containers/overview), which allows you to start/stop the container and additionally attach VSCode to the container by right-clicking on the container and selecting `Attach Visual Studio Code`.

### Maplab Install

First, install a python dependency by running the following commands from the root of the ros workspace:
```
cd src/maplab/dependencies/3rdparty/robotdataprocess/
unset PYTHONPATH
source /opt/miniconda3/bin/activate robotdataprocess
pip install rospkg
pip install .
```

Then, close the terminal to deactivate the `robotdataprocess` environment.

Finally, run these commands from the root directory of the ros workspace:
```
catkin init
catkin config --merge-devel
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build maplab
```

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

### Hercules Dataset

First, make changes to the following variables (to update data and user directory names):
```
hercules_bags_folder (applications/maplab-server-node/launch/hercules/hercules-maplab-server-robots.launch)
hercules_root_map_directory (applications/maplab-server-node/launch/hercules/hercules-maplab-server-robots.launch)
```

Then run the following command to run maplab:
```
tmuxp load src/maplab/applications/maplab-server-node/launch/hercules/hercules_tmuxp_launch.yaml
```
