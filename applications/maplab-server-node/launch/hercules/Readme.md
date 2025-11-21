# How to run Maplab with Hercules Dataset



## Step1: Preprocess Data
- Process Hercules data with simple_extract_to_bag.py inside [robotdataprocess]([https://github.com/lunarlab-gatech/robotdataprocess](https://github.com/lunarlab-gatech/robotdataprocess/blob/develop/examples/Hercules/simple_extract_to_bag.py))

## Step2: Modify Files

Files to modify: 
- **hercules-maplab-server-robots.launch**: "hercules_bags_folder" set to data directory
- **hercules/calib** camera configuration files

## Step3: Run the experiment

[Maplab Server — EuRoC Experiment Tutorial](https://maplab.asl.ethz.ch/docs/master/pages/tutorials-maplab-server/B_Euroc_Experiment.html)
