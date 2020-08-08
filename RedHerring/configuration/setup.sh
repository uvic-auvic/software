# This script installs the full desktop suite of ROS (Robotic Operating System) on Ubuntu 14.04 based systems
# It should be run as root in order to run uninhibited

# Add ROS, Gazebo7 and Blender to sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
sudo add-apt-repository ppa:irie/blender

# Add Package Keys to list
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# Update package list
sudo apt-get update

# Install whole ROS desktop application suite + gazebo 7
sudo apt-get install -y ros-indigo-desktop-full 
sudo apt-get remove -y gazebo2
sudo apt-get install -y ros-indigo-gazebo7-ros-pkgs ros-indigo-gazebo7-ros-control ros-indigo-gazebo7-plugins ros-indigo-gazebo7-msgs libgazebo7 libgazebo7-dev

# Install all the required packages
source package-setup.sh

# Initialize ROSDEP
sudo rosdep init
rosdep update

# Replace symlink with file
pushd ../ros/src
FILENAME="CMakeLists.txt"
FILE_LOC=`readlink -f $FILENAME`
rm $FILENAME
cp $FILE_LOC $FILENAME
popd

# Setup environment
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install rosinstall
sudo apt-get install -y python-rosinstall

# Install Blender
sudo apt-get install blender
