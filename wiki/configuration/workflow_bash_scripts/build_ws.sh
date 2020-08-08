ROS_DISTRO="melodic"

source /opt/ros/${ROS_DISTRO}/setup.bash
cd ../../trident

rosdep install --from-paths src --ignore-src -r -y
catkin_make