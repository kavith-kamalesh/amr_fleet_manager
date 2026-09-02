#!/bin/bash
# Usage: ./assign_task.sh [robot_id] [task_type] [goal_x] [goal_y]
# Example: ./assign_task.sh robot1 urgent 10.5 5.2

ROBOT_ID=$1
TASK_TYPE=$2
GOAL_X=$3
GOAL_Y=$4

if [ "$#" -ne 4 ]; then
    echo "Error: Incorrect number of arguments."
    echo "Usage: ./assign_task.sh [robot_id] [task_type] [goal_x] [goal_y]"
    exit 1
fi

echo "Reconfiguring kinematics for [$ROBOT_ID] -> Task: [$TASK_TYPE]"

case $TASK_TYPE in
  "fragile")
    # 50% Speed/Accel reduction + wider safety margin
    ros2 param set /$ROBOT_ID/controller_server max_vel_x 0.3
    ros2 param set /$ROBOT_ID/controller_server max_accel_x 0.5
    ros2 param set /$ROBOT_ID/local_costmap/inflation_layer inflation_radius 1.0
    ;;
    
  "urgent")
    # Maximum aggressive Speed/Accel + tighter cornering
    ros2 param set /$ROBOT_ID/controller_server max_vel_x 1.5
    ros2 param set /$ROBOT_ID/controller_server max_accel_x 2.5
    ros2 param set /$ROBOT_ID/local_costmap/inflation_layer inflation_radius 0.3
    ;;
    
  "low_battery")
    # Constant slow speed to minimize power spikes
    ros2 param set /$ROBOT_ID/controller_server max_vel_x 0.2
    ros2 param set /$ROBOT_ID/controller_server max_accel_x 0.2
    ;;
    
  *)
    echo "Error: Unknown task type. Use 'fragile', 'urgent', or 'low_battery'."
    exit 1
    ;;
esac

echo "=========================================="
echo "Preempting route... Dispatching: X=$GOAL_X, Y=$GOAL_Y"
echo "=========================================="

ros2 action send_goal /$ROBOT_ID/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {w: 1.0}}}}"
