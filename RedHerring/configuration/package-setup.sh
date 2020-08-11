#!/bin/bash

##########################################
# Installs all the required ROS packages #
# found in the ros-packages.list         #
##########################################

# Filename
PACK_FILENAME="ros-packages.list"

# Iterate over lines in file and place all into formatted string
while read -r line
do
    PACKAGES="ros-indigo-$line $PACKAGES"
done < $PACK_FILENAME

# install all the applications in a single apt-get call
sudo apt-get install -y $PACKAGES
