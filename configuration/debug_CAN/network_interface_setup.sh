#!/bin/bash

# This script setups the socketCAN bus

echo What do you want to call the virtual bus?
read BUSNAME

sudo ip link add dev $BUSNAME type vcan
sudo ip link set up $BUSNAME

echo done. 
