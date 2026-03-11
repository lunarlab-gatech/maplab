LOG_DIR="/home/nisemono/.ros/log/c0c72e28-ed91-11f0-a982-94b6095fff8c"

  echo "=== Checking maplab_node startup ==="
  grep -E "Visual|Lidar|ENABLED|DISABLED" "$LOG_DIR"/Husky1-maplab_node-4*.log | head -20

  echo -e "\n=== Checking map output folder setting ==="
  grep "map_output_folder\|Set VIMap folder" "$LOG_DIR"/Husky1-maplab_node-4*.log

  echo -e "\n=== Checking if map was saved ==="
  grep -E "Saving map|Saved map|empty.*nothing" "$LOG_DIR"/Husky1-maplab_node-4*.log | tail -10

  echo -e "\n=== Checking vertex creation ==="
  grep -E "Applied VIO update|numVertices|Created.*vertex" "$LOG_DIR"/Husky1-maplab_node-4*.log | wc -l

  echo -e "\n=== Checking lidar attachment ==="
  grep -E "Attached.*lidar|Attached raw lidar" "$LOG_DIR"/Husky1-maplab_node-4*.log | wc -l

  echo -e "\n=== Finding any vi_map or resource folders ==="
  find ~ -name "vi_map" -o -name "*resource*" -type d 2>/dev/null | grep -i husky