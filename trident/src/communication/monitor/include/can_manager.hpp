// NOTE: if you get red line sunder the types, you need to locate them by following the include path
#ifndef DEVICE_MANAGER_HPP
#define DEVICE_MANAGER_HPP

//C++ library
#include <vector>
#include <exception>
#include <map>
#include <string>

// ROS kernal
#include <ros/publisher.h>
#include <ros/service.h>
#include <ros/subscriber.h>
#include <ros/console.h>
#include "can_msgs/Frame.h"

//auvic library
#include "monitor/GetDeviceMessage.h"

using GetDeviceMessageReq = monitor::GetDeviceMessage::Request;
using GetDeviceMessageRes = monitor::GetDeviceMessage::Response;

class Can_Manager{
    public:
        Can_Manager(
            ros::NodeHandle* n_auvic,
            ros::NodeHandle* n_socketcan
        );
        void From_Can(const can_msgs::Frame::ConstPtr& msg);
        // TODO: change input paramters to match messages
        bool To_Can(GetDeviceMessageReq& req, GetDeviceMessageRes& res);
        // TODO: Assemble can_msg for socketcan_bridge
        can_msgs::Frame Make_Can_msg();
        // TODO: check which devices are connected to the CAN bus
        void Startup_routine(ros::NodeHandle* n_auvic);
        // TODO: Check a single device status on the CAN bus
        void Check_device_status(std::string device_name);
        // TODO: Check all devices on the CAN bus
        void Check_all_devices_status();
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