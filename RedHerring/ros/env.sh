#!/bin/bash

#TODO: Add a -h or --help argument

#CONSTANTS
DEFAULT_HOST="localhost"
DEFAULT_IP=`hostname -I`
ROS_DEFAULT_PORT=11311

# We only want 1 argument, anything more is invalid
if [[ $# -gt 1 ]]
then
    echo "Too many arguments"
    exit 1
fi

# Set the default hostname, if no arguments are passed in
HOST=$DEFAULT_HOST
PORT=$ROS_DEFAULT_PORT

# This environment varaible will be read by the launch file
# And determine which packages to launch
# If it's local, it will launch the local nodes with local settings
# In the event that this name conflicts with something in the ROS official namespace then change it
ROS_MASTER_LOC="local"

# TODO: Replace this with a regex test to test
#       whether the argument is in valid IP address format
# XXX.XXX.XX.XX
if [[ $# -eq 1 ]]
then
    HOST=$1
    ROS_MASTER_LOC="remote"
fi

export ROS_MASTER_URI="http://$HOST:$PORT"
export ROS_IP="${DEFAULT_IP// /}"
export ROS_MASTER_LOC

echo "Using ROS_MASTER_URI $ROS_MASTER_URI"
echo "Using ROS_IP $ROS_IP"

source devel/setup.bash
