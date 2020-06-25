#ifndef DEVICE_MANAGER_HPP
#define DEVICE_MANAGER_HPP

//C++ library
#include <vector>
#include <exception>
#include <map>
#include <string>

// boost library
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

// ROS kernal
#include <ros/ros.h>
#include <ros/console.h>
#include "can_msgs/Frame.h"

//auvic library
#include "monitor/GetADevice.h"
#include "monitor/GetAllDevices.h"
#include "monitor/GetCanMSG.h"

using GetAReq = monitor::GetADevice::Request;
using GetARes = monitor::GetADevice::Response;
using GetAllReq = monitor::GetAllDevices::Request;
using GetAllRes = monitor::GetAllDevices::Response;
using GetCanMsgReq = monitor::GetCanMSG::Request;
using GetCanMsgRes = monitor::GetCanMSG::Response;
using ptree = boost::property_tree::ptree;

class DeviceNotFoundException : public std::runtime_error {
public:    
    DeviceNotFoundException(const std::string & dev) :
        std::runtime_error("Could not find device \"" + dev + "\"")
    {}
};

/* The JSON file gets parsed I assume sequentially as listed below */
class Device_Property {
    public:
        // constructor
        Device_Property(
            std::string name, uint8_t id, bool ignore, std::string ack_m, std::string ack_r, int baud, int timeout, 
            int retry_count, bool convert_to_bytes, size_t size_of_message, size_t size_of_response,
            bool big_endian_message, bool big_endian_response) :
        name(name),
        ignore(ignore),
        ack_message(ack_m),
        ack_response(ack_r),
        baud(baud),
        convert_to_bytes(convert_to_bytes),
        retry_count(retry_count),
        size_of_response(size_of_response),
        size_of_message(size_of_message),
        big_endian_message(big_endian_message),
        big_endian_response(big_endian_response) {}
        
        // Parameters
        std::string name;
        uint8_t id;
        bool ignore;
        std::string ack_message;
        std::string ack_response;
        int baud;
        int retry_count;
        bool convert_to_bytes;
        size_t size_of_message;
        size_t size_of_response;
        bool big_endian_message;
        bool big_endian_response;
};

namespace monitor 
{

class Can_Manager{
    public:
        Can_Manager(
            const std::vector<Device_Property> & properties,
            ros::NodeHandle* n_auvic);

        void setup(const std::vector<Device_Property> & properties);
        
        bool get_all_devices(GetAllReq &, GetAllRes &);
        bool get_a_device(GetAReq &, GetARes &);
        void THECALLfromTheBUS(const can_msgs::Frame::ConstPtr& msg);
        bool toCan(GetCanMsgReq& req, GetCanMsgRes& res);
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
        
        std::vector<Device_Property> JSON_Device_List;
        std::map<std::string, Device_Property> devices;
};
}


#endif /* DEVICE_MANAGER_HPP */