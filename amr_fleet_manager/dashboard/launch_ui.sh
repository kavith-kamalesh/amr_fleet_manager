#!/bin/bash
echo "Starting ROSBridge Server..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ROS_PID=$!

echo "Serving Jury Dashboard on http://localhost:8000"
python3 -m http.server 8000
kill $ROS_PID
