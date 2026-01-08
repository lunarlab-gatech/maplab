#!/bin/bash
echo "=== What is maplab_node subscribed to? ==="
echo ""
echo "Finding maplab_node:"
rosnode list | grep maplab
echo ""
echo "Subscriptions:"
rosnode info /Husky1/maplab_node 2>/dev/null | grep -A 100 "Subscriptions:" | head -30
echo ""
echo "=== Checking if sensor_calibration_file parameter is set ==="
rosparam get /Husky1/maplab_node/sensor_calibration_file 2>/dev/null || echo "  → Parameter NOT set!"
echo ""
echo "=== Checking datasource_type ==="
rosparam get /Husky1/maplab_node/datasource_type 2>/dev/null || echo "  → Parameter NOT set!"
