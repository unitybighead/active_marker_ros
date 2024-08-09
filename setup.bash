#!/bin/bash

if [ "$(whoami)" = "root" ]; then
  echo "Please don't run as root"
  exit 255
fi

# get ROBOT_ID
echo "Enter the ROBOT_ID"
read ROBOT_ID

# enter TEAM_COLOR
echo "Enter the TEAM_COLOR"
read TEAM_COLOR

# create config.txt
echo \
"TEAM_COLOR=$TEAM_COLOR
ROBOT_ID=$ROBOT_ID" > config.txt

exit 0