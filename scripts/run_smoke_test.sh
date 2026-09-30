#!/usr/bin/env bash
# Usage: ./scripts/run_smoke_test.sh [duration_sec] [label]
# Runs mutex_smoke.launch.py under ROS 2 (RoboStack env). The recorder stops itself, so nothing
# depends on signals reaching background jobs. Logs go to ~/smoke_logs/<label>.*
DURATION="${1:-40}"; LABEL="${2:-run}"
REPO="$HOME/amr_fleet_manager"
unset VIRTUAL_ENV PYTHONPATH PYTHONHOME
eval "$(micromamba shell hook --shell bash)"
micromamba activate ros_jazzy || { echo "cannot activate ros_jazzy"; exit 1; }
source "$HOME/ros_ws/install/setup.bash" || { echo "cannot source ros_ws install"; exit 1; }
echo "executables installed: $(ros2 pkg executables amr_fleet_manager | wc -l)"

STRAYS="ros2 launch amr_fleet_manager|ros2 bag record|install/amr_fleet_manager/lib|smoke_odom.py|sim_lidar.py"
pkill -TERM -f "$STRAYS" 2>/dev/null; sleep 2; pkill -KILL -f "$STRAYS" 2>/dev/null
rm -f /tmp/smoke_launch.log /tmp/smoke_odom.csv /tmp/smoke_odom_events.csv /tmp/graph_snapshot.txt /tmp/rec.log
mkdir -p "$HOME/smoke_logs"

python "$REPO/scripts/smoke_odom.py" record --duration "$DURATION" --out /tmp/smoke_odom.csv > /tmp/rec.log 2>&1 &
REC=$!
sleep 2
ros2 launch amr_fleet_manager mutex_smoke.launch.py > /tmp/smoke_launch.log 2>&1 &
LAUNCH=$!
# LIDAR=1 adds a geometry-driven fake LiDAR so safety_fallback / the e-stop reroute actually run.
if [ "$LIDAR" = "1" ]; then python "$REPO/scripts/sim_lidar.py" > /tmp/sim_lidar.log 2>&1 & fi
# Graph snapshot at ~20 s: duplicate node names or extra publishers would show up here.
( sleep 20; { ros2 node list; for t in odom cmd_vel goal_pose; do echo "-- /robot3/$t"; ros2 topic info /robot3/$t; done; } > /tmp/graph_snapshot.txt 2>&1 ) &
SNAP=$!
wait $REC
wait $SNAP 2>/dev/null
kill -TERM $LAUNCH 2>/dev/null; sleep 8
pkill -KILL -f "$STRAYS" 2>/dev/null

cp /tmp/smoke_launch.log "$HOME/smoke_logs/$LABEL.log"; cp /tmp/smoke_odom.csv "$HOME/smoke_logs/$LABEL.csv" 2>/dev/null
cp /tmp/smoke_odom_events.csv "$HOME/smoke_logs/$LABEL.events.csv" 2>/dev/null
cp /tmp/graph_snapshot.txt "$HOME/smoke_logs/$LABEL.graph.txt" 2>/dev/null
echo "=== recorder ==="; cat /tmp/rec.log
echo "=== events ==="
grep -E "Dispatched|New FMS goal|Goal reached|Rerouting|Physically|EMERGENCY|Front sector clear|Traceback" /tmp/smoke_launch.log | cut -c1-190 | head -40
echo "reroute events: $(grep -ci 'rerouting around' /tmp/smoke_launch.log)   'no alternate route' lines: $(grep -c 'No alternate route' /tmp/smoke_launch.log)"
echo "=== separation ==="
python "$REPO/scripts/smoke_odom.py" analyze --csv /tmp/smoke_odom.csv
echo "e-stop engagements: $(grep -c 'EMERGENCY STOP' /tmp/smoke_launch.log)   releases: $(grep -c 'releasing emergency stop' /tmp/smoke_launch.log)"
