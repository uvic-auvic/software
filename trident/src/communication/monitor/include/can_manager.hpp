// NOTE: if you get red line sunder the types, you need to locate them by following the include path
#ifndef DEVICE_MANAGER_HPP
#define DEVICE_MANAGER_HPP

//C++ library
#include <vector>
#include <exception>
#include <algorithm>
#include <string>
#include <iostream>

// ROS kernal
#include <ros/publisher.h>
#include <ros/service.h>
#include <ros/subscriber.h>
#include <ros/console.h>
#include "can_msgs/Frame.h"

//auvic library
#include "protocol.h"
#include "auvic_msgs/devices_to_monitor.h"

using GetMonitorReq = auvic_msgs::devices_to_monitor::Request;
using GetMonitorRes = auvic_msgs::devices_to_monitor::Response;

class Can_Manager{
    public:
        Can_Manager(
            ros::NodeHandle* n_auvic,
            ros::NodeHandle* n_socketcan
        );
        void From_Can(const can_msgs::Frame::ConstPtr& msg);

        void message_handle(ros::Publisher Peripheral_topic_handle, std::string topic, can_msgs::Frame msg);
        // TODO: change input paramters to match messages
        bool To_Can(GetMonitorReq& req, GetMonitorRes& res);
        // TODO: Assemble can_msg for socketcan_bridge
        void Make_Can_msg(GetMonitorReq& input, can_msgs::Frame& output);
        // TODO: check which devices are connected to the CAN bus
        void Startup_routine(ros::NodeHandle* n_auvic);
        // TODO: Check a single device status on the CAN bus
        void Check_device_status(std::string device_name);
        // TODO: Check all devices on the CAN bus
        void Check_all_devices_status();
        // TODO: do a CRC
        void CRC();
    private:
        ros::Publisher send_;
        ros::ServiceServer node_server_;
        ros::Subscriber receive_;

        ros::Publisher Peripheral_torpedo;
        ros::Publisher Peripheral_powerboard;
        ros::Publisher Peripheral_motorcontroller;
        ros::Publisher Peripheral_lcd_board;
        ros::Publisher Peripheral_dvl;
        ros::Publisher Peripheral_imu;
        ros::Publisher Peripheral_grabber;
        ros::Publisher Peripheral_dropper;
        ros::Publisher Peripheral_hydrophone;
        
};


#endif /* DEVICE_MANAGER_HPP */