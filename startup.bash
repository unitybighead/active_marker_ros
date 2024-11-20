#!/bin/bash
cd "$(dirname "$0")"
. install/setup.bash
. config.txt
# if [ $ROBOT_ID -ge 10 ]; then
#     export ROS_DOMAIN_ID=1"$ROBOT_ID"
# else
#     export ROS_DOMAIN_ID=10"$ROBOT_ID"
# fi
# export ROS_LOCALHOST_ONLY=1

echo $ROBOT_ID 
echo $TEAM_COLOR

ros2 run active_marker illuminance_pub  __ns:=/am"$ROBOT_ID" &
ros2 run active_marker illuminance_sub __ns:=/am"$ROBOT_ID" &
ros2 run active_marker color_sub __ns:=/am"$ROBOT_ID" &
ros2 run active_marker robot_ID __ns:=/am"$ROBOT_ID" --ros-args -p ID:="$ROBOT_ID" -p team_color:="$TEAM_COLOR" &

wait
