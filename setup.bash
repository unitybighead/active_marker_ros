#!/bin/bash

if [ "$(whoami)" = "root" ]; then
  echo "Please don't run as root"
  exit 255
fi

wget https://raw.githubusercontent.com/karaage0703/jetson-nano-tools/master/udev_rules/55-tegraserial.rules
sudo mv 55-tegraserial.rules /etc/udev/rules.d/
sudo usermod -a -G tty $(whoami)
sudo usermod -a -G dialout $(whoami)
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty
sudo chmod 660 /dev/ttyTHS1
sudo chmod 660 /dev/ttyTHS2
echo "Please reboot"
