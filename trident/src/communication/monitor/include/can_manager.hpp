// NOTE: if you get red line sunder the types, you need to locate them by following the include path
#ifndef DEVICE_MANAGER_HPP
#define DEVICE_MANAGER_HPP

//C++ library
#include <vector>
#include <map>
#include <exception>
#include <algorithm>
#include <string>
#include <iostream>

// ROS kernal
#include <ros/ros.h>
#include <ros/console.h>
#include "can_msgs/Frame.h"

//auvic library
#include "auvic_msgs/Monitor_Jetson_To_World.h"

class SoftwareToHardware {
public:
    SoftwareToHardware(ros::NodeHandle & n, ros::NodeHandle & nh);
    void Jetson_To_World(const auvic_msgs::Monitor_Jetson_To_World::ConstPtr& msg);
private:
    ros::Publisher Send_The_Message_To_The_World;
    ros::Subscriber TRIDENT_deviceName;
    ros::Subscriber TRIDENT_motorSetSpeed;
    ros::Subscriber TRIDENT_powerEnable;
    ros::Subscriber TRIDENT_MCISOTP;
    
}; // End of class 'SoftwareToHardware'

class HardwareToSoftware {
public:
    HardwareToSoftware(ros::NodeHandle & n, ros::NodeHandle & nh);
    void World_To_Jetson(const can_msgs::Frame::ConstPtr& msg);
private:
    ros::Subscriber Get_The_Message_From_The_World;
    ros::Publisher MC_deviceName;
    ros::Publisher MC_motorRPMLow;
    ros::Publisher MC_motorRPMHigh;
    ros::Publisher MC_ISOTP;
    ros::Publisher PB_deviceName;
    ros::Publisher PB_envData;
    ros::Publisher PB_battVoltages;
    ros::Publisher PB_battCurrents;


}; // End of class 'SoftwareToHardware'

#endif /* DEVICE_MANAGER_HPP */