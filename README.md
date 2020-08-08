# Autonomous Underwater Vehicles IC Software Repository

![auvic-trident-ci](https://github.com/uvic-auvic/software/workflows/auvic-trident-ci/badge.svg?branch=master)

Welcome to AUVIC's software repository. We use the Robot Operating System (ROS) to build C++ and python programs that give our autonomous underwater vehicles life. Computer vision, artificial Intelligence, motor controls, neural networks, web development, and UI/UX design are some of many projects we pursue. If you are interested in joining our team or have any questions or just wanna talk, email us @ auvic@uvic.ca :)

# Table of Contents

1. [Getting Started](#introduction)
2. [File Directories](#File_Directories)
    1. [Trident](#trident)
    2. [Polaris](#polaris)
    3. [Red Herring](#rh)
    4. [wiki](#wiki)
    5. [.github/workflows](#ci)
    6. [Software Sandbox](#ss)

# Getting Started

1. Fork this repository
2. Clone your copy onto your computer via GIT
    - When installing GIT, make sure to set in the settings "commit as-is". This will save us headaches in the future.
3. Install ROS Noetic (or Melodic)
    - ROS runs on Ubuntu 20.04 (Noetic) and Ubuntu 18.04 (Melodic), however, WSL (windows subsystem) has worked with ROS Kinetic, but I am unsure if this still is the case.
    - Docker is another alternative, however, that is beyond the scope of this "getting started" since it doesn't run on all computers.
    - I created a bash script in wiki/configuration/system_setup called "noetic_setup.sh" for Ubuntu 20.04 users that will install ros & rosdep for the lazy ones.
4. Go into "trident" and call rosdep on each of the packages to install the external dependencies. 

Email auvic@uvic.ca or contact the software lead for help.

# File Directories

## Trident <a name="trident"></a>

The 2021 auv that is currently in development. To create a pull request, Squash your commits by:
1. git reset --soft HEAD~< number of commits >
    - git reset --soft HEAD~27
2. git commit -m "message"
3. git push origin +name-of-branch
    - The plus sign is not a typo

## Polaris <a name="polaris"></a>

The 2018 auv that is now getting retired.

## Red Herring <a name="rh"></a>

The 2016 auv that has been retired.

## Wiki <a name="wiki"></a>

Store your images, rqt_graphs, and URDF/Gazebo screenshots here. 

## .github/workflows <a name="ci"></a>

Contains CI file for trident. Check it out!

## Software Sandbox <a name="ss"></a>

Has some sandbox stuff. Check it out, not sure whats really inside honestly.