#ifndef SAILORMAN_HPP
#define SAILORMAN_HPP

#include <peripheral_manager.hpp>
#include <auvic_msgs/Monitor_Send_Data_To_World.h>
#include <auvic_msgs/Devices_Get_Data_From_Peripherals.h>

using Send_Data_Request = auvic_msgs::Monitor_Send_Data_To_World::Request;
using Send_Data_Response = auvic_msgs::Monitor_Send_Data_To_World::Response;
using Get_Data_Request = auvic_msgs::Devices_Get_Data_From_Peripherals::Request;
using Get_Data_Response = auvic_msgs::Devices_Get_Data_From_Peripherals::Response;

class SailorMan {
public:
    SailorMan(ros::NodeHandle & n_auvic, ros::NodeHandle & server);
    SailorMan(ros::NodeHandle & n_auvic);
    ~SailorMan();

    // Services to send and receive data, respectively.
    ros::ServiceServer To_World_Service, Get_Data_Service;
    bool Send_Data_To_World(Send_Data_Request &req, Send_Data_Response &res);
    bool Get_Data_From_Peripherals(Get_Data_Request &req, Get_Data_Response &res);

    // Gets triggered every 10 seconds to reconfirm device list
    std::vector<std::string> list_of_publishing_topics;//contains periodic topics
    ros::Timer NM_time;
    void Run_Network_Management(const ros::TimerEvent&);

    // In future, when we have more messages, we may want to encapsilate publishers into a vector.
    ros::Publisher TRIDENT_deviceName, TRIDENT_motorSetSpeed, TRIDENT_powerEnable, TRIDENT_MCISOTP; 
};

#endif /* SAILORMAN_HPP */