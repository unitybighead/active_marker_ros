#!/bin/bash

if [ "$(whoami)" = "root" ]; then
  echo "Please don't run as root"
  exit 255
fi

cd "$(dirname "$0")"

# get ROBOT_ID
echo "Enter the ROBOT_ID"
read ROBOT_ID

# Writing automatic startup and registering the service
echo \
"[Unit]
Description=\"active_marker_startup\"
After = systemd-networkd-wait-online.service

[Service]
Type=simple
ExecStart=\"$(pwd)/startup.bash\"
Restart=always
Environment=\"HOME=root\"

[Install]
WantedBy=multi-user.target" \
| sudo tee "/lib/systemd/system/active_marker.service" > /dev/null && \
chmod 744 "$(pwd)/startup.bash" && \
sudo systemctl enable active_marker

# install apt that needed by packages
rosdep update
rosdep install -i --from-paths src/*

# create config.txt
echo \
"TEAM_COLOR=yellow
ROBOT_ID=$ROBOT_ID" > config.txt

# UART settings
wget https://raw.githubusercontent.com/karaage0703/jetson-nano-tools/master/udev_rules/55-tegraserial.rules
sudo mv 55-tegraserial.rules /etc/udev/rules.d/
sudo usermod -a -G tty $(whoami)
sudo usermod -a -G dialout $(whoami)
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty
sudo chmod 666 /dev/ttyTHS1
sudo chmod 666 /dev/ttyTHS2

exit 0
