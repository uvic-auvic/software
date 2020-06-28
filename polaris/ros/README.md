# Welcome to the ROS Code

## Building

The project can be build by envoking `catkin\_make` in this directory. (This requires the install procedure to be carried out on a Ubuntu 16.04 machine.)

The other way that this project can be built is through docker by envoking `./docker_make.sh`. It is to note that this doesn't work is because it's dependent on docker being installed and the user account envoking the script has been added to the docker group `sudo usermod -aG docker $USER` and a logout/login has been performed to refresh the users groups. If any other troubles are experienced contact avlecxk (at) gmail (dot) com.
