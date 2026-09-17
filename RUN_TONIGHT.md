# On Ubuntu machine — run in this exact order

cd ~/amr_ws/src/amr_fleet_manager
git pull origin main
grep -n "stop_and_wait_node" setup.py    # must show the registration line

cd ~/amr_ws
colcon build --packages-select amr_fleet_manager --symlink-install
source install/setup.bash
ros2 pkg executables amr_fleet_manager | grep stop_and_wait   # must print the line

# --- Run A: baseline ---
mkdir -p bags && cd bags
ros2 bag record -o baseline_run /robot1/odom /robot2/odom /robot3/odom /robot4/odom /robot5/odom /robot6/odom &
BAG_PID=$!
# launch central_dispatcher + stop_and_wait_node per robot (NOT waypoint_nav_node/spatial_mutex)
# wait for all 6 to print ARRIVED on baseline_status, then:
kill $BAG_PID
cd ..
python3 scripts/measure_collisions.py bags/baseline_run --radius 0.35

# --- Run B: your real coordinated system ---
cd bags
ros2 bag record -o coordinated_run /robot1/odom /robot2/odom /robot3/odom /robot4/odom /robot5/odom /robot6/odom &
BAG_PID=$!
# launch waypoint_nav_node + spatial_mutex per robot, same goals as before
kill $BAG_PID
cd ..
python3 scripts/measure_collisions.py bags/coordinated_run --radius 0.35

## Health scoring feature — verify after colcon build

ros2 pkg executables amr_fleet_manager | grep -E "battery_monitor|health_monitor|alert_dispatcher"
# should print all three

# Launch file now includes battery_monitor + health_monitor per robot,
# and alert_dispatcher once fleet-wide. No separate launch step needed
# if using hybrid_fleet_bringup.launch.py as before.
