#!/bin/bash

# This script installs can-utils - the socketCAN interface.

echo 'getting socketCAN setup...'
sudo apt-get install can-utils

# these commands must be run everytime a terminal is started
echo "sudo modprobe vcan" >> ~/.bashrc
echo "sudo modprobe can" >> ~/.bashrc
echo "sudo modprobe slcan" >> ~/.bashrc

source ~/.bashrc

