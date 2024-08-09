#!/bin/bash

if [ "$(whoami)" = "root" ]; then
  echo "Please don't run as root"
  exit 255
fi

# check OS version
source /etc/lsb-release

# set ROS_DISTRO
if [ "$DISTRIB_RELEASE" == "18.04" ]
then
  ROS_DISTRO="dashing"
elif [ "$DISTRIB_RELEASE" == "20.04" ]
then
  ROS_DISTRO="foxy"
elif [ "$DISTRIB_RELEASE" == "22.04" ]
then
  ROS_DISTRO="humble"
else
  echo "Unsaported Ubuntu version"
  exit 255
fi

echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# install apt that needed by packages
rosdep update
rosdep install -i --from-paths src/*

# UART settings
wget https://raw.githubusercontent.com/karaage0703/jetson-nano-tools/master/udev_rules/55-tegraserial.rules
sudo mv 55-tegraserial.rules /etc/udev/rules.d/
sudo usermod -a -G tty $(whoami)
sudo usermod -a -G dialout $(whoami)
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty
sudo chmod 666 /dev/ttyTHS1
sudo chmod 666 /dev/ttyTHS2

# Write automatic startup and registering the service
echo "[Unit]
Description=active_marker_startup
After=network.target

[Service]
Type=simple
ExecStart=/bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && $(pwd)/startup.bash'
Restart=always
Environment=HOME=/root

[Install]
WantedBy=multi-user.target" | sudo tee /etc/systemd/system/active_marker.service


sudo systemctl daemon-reload
sudo systemctl enable active_marker.service
sudo systemctl start active_marker.service

echo "Please reboot"
