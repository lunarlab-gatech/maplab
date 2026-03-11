#!/bin/bash
echo "=== Checking odometry topic connection ==="
echo ""
echo "1. Topic info (should show both publisher AND subscriber):"
rostopic info /Husky1/rovioli/maplab_odom_T_M_I
echo ""
echo "2. Message type:"
rostopic type /Husky1/rovioli/maplab_odom_T_M_I
echo ""
echo "3. Is data flowing? (2 second test)"
timeout 2 rostopic hz /Husky1/rovioli/maplab_odom_T_M_I 2>&1 || echo "  → Topic exists but may not be publishing"
echo ""
echo "4. Sample message:"
timeout 1 rostopic echo /Husky1/rovioli/maplab_odom_T_M_I -n 1 2>&1 | head -20 || echo "  → No messages received"
