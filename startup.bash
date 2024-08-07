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

ros2 run active_marker illuminance_pub __params:=config.yaml __ns:=/am"$ROBOT_ID" &
ros2 run active_marker illuminance_sub __params:=config.yaml __ns:=/am"$ROBOT_ID" &
ros2 run active_marker color_sub __params:=config.yaml __ns:=/am"$ROBOT_ID" &

wait