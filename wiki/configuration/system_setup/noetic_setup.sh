#!/bin/bash

# This script installs the full desktop suite of ROS Noetic on Ubuntu 20.04
# It should be run as root in order to run uninhibited

ROSVERSION="noetic"
ROSDEP_PYTHON="python3"
# Add ROS Package to list with keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install the core ROS programs
sudo apt-get update
sudo apt-get install -y ros-$ROSVERSION-desktop-full

# Install rosdep
sudo apt-get install ${ROSDEP_PYTHON}-rosdep
sudo rosdep init
rosdep update

# Setup environment. Comments out if you don't use bash
echo "source /opt/ros/$ROSVERSION/setup.bash" >> ~/.bashrc
source ~/.bashrc
mkdir -p ~/rosbag

