#!/bin/bash
EDGE_IP=$1
if [ -z "$EDGE_IP" ]; then
    echo "Error: Provide edge device IP (e.g., ubuntu@192.168.1.50)"
    exit 1
fi

echo "Packaging AMR Fleet Manager..."
COPYFILE_DISABLE=1 tar --exclude='__pycache__' -czf edge_deploy.tar.gz amr_fleet_manager resource launch worlds setup.py setup.cfg package.xml

echo "Transferring to Edge Device: $EDGE_IP..."
scp edge_deploy.tar.gz $EDGE_IP:~/

echo "Executing Remote Build..."
ssh $EDGE_IP << 'REMOTE'
    mkdir -p ~/amr_ws/src/amr_fleet_manager
    tar -xzf edge_deploy.tar.gz -C ~/amr_ws/src/amr_fleet_manager
    cd ~/amr_ws
    source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
    colcon build --packages-select amr_fleet_manager --symlink-install
    echo "Build Complete on Edge."
REMOTE
rm edge_deploy.tar.gz
