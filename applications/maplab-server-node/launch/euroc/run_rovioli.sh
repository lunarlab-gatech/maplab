# source ~/maplab_ws/devel/setup.bash
# source /opt/ros/melodic/setup.bash
export LOCALIZATION_MAP_OUTPUT=/data/euroc/euroc_maps_5
rosrun rovioli rovioli \
  --alsologtostderr=1 \
  --v=2 \
  --sensor_calibration_file=/root/maplab_ws/src/maplab/applications/maplab-server-node/launch/euroc/calib/euroc-stereo-mh5.yaml \
  --external_imu_parameters_rovio=$ROVIO_CONFIG_DIR/imu-adis16488-2.yaml \
  --datasource_type="rostopic" \
  --save_map_folder=$LOCALIZATION_MAP_OUTPUT \
  --map_builder_save_image_as_resources=false \
  --optimize_map_to_localization_map=false \
  --vio_nframe_sync_tolerance_ns=1000000 \