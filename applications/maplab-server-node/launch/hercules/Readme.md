# How to run Maplab with Hercules Dataset



## Step1: Preprocess Data
- Process Hercules data with simple_extract_to_bag.py inside [robotdataprocess](https://github.com/lunarlab-gatech/robotdataprocess/blob/develop/examples/Hercules/simple_extract_to_bag.py)

## Step2: Modify Files

Files to modify: 
- **hercules-maplab-server-robots.launch**: "hercules_bags_folder" set to data directory
- **hercules/calib** camera configuration files

## Step3: Run the experiment

tmuxp load hercules_tmuxp_launch.yaml

## Optional: Enabling or Disabling Lidar features
- Enabling:   
    - Set maplab_server_enable_lidar_loop_closure: true
    - Set map_builder_save_point_clouds_as_resources: true
- Disabling (reverse changes)
