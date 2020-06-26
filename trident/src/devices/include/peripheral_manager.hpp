#ifndef PERIPHERAL_MANAGER_HPP
#define PERIPHERAL_MANAGER_HPP
//C++ library

// ROS kernal
#include <ros/ros.h>
#include <ros/console.h>
#include "can_msgs/Frame.h"

//auvic library
#include "monitor/GetCanMSG.h"

class Peripherals{
    public:
        Peripherals (){};
        ~Peripherals (){};
        void topic_callback(const can_msgs::Frame::ConstPtr& msg);
        // TODO: construct can_msgs frame from data
        bool send_frame(const can_msgs::Frame::ConstPtr& data);
    
        // peripheral is turned off by default
        bool ignore = 1;
        monitor::GetCanMSG srv;
        can_msgs::Frame msg;
        uint8_t data[8];
        ros::Subscriber sub;
        ros::ServiceClient client;
};

class Powerboard: public Peripherals{
    public:
        Powerboard(ros::NodeHandle* n_auvic);
        ~Powerboard();
        void topic_callback(const can_msgs::Frame::ConstPtr& msg);
        // TODO: get_powerboard_data: will create a can_msgs file and pass to 'send_frame'
        bool get_powerboard_data();
        // TODO: power_enabler: see polaris for implementation
        bool power_enabler();
        // TODO: average_ext_pressure: see polaris for implementation
        bool average_ext_pressure();

        // protocol_deviceName_S    PB_deviceName; // Sent by Power Board, Received by Polaris
        // protocol_PBEnvData_S     PB_envData; // Sent by Power Board, Received by Polaris
        // protocol_PBBattVoltages_S PB_battVoltages; // Sent by Power Board, Received by Polaris
        // protocol_PBBattCurrents_S PB_battCurrents; // Sent by Power Board, Received by Polaris
};

class Motorcontroller: public Peripherals{
    public:
        Motorcontroller(ros::NodeHandle* n_auvic);
        ~Motorcontroller();
        void topic_callback(const can_msgs::Frame::ConstPtr& msg);
    private:
};
class Imu: public Peripherals{
    public:
        Imu(ros::NodeHandle* n_auvic);
        ~Imu();
        void topic_callback(const can_msgs::Frame::ConstPtr& msg);
    private:
};
class Grabber: public Peripherals{
    public:
        Grabber(ros::NodeHandle* n_auvic);
        ~Grabber();
        void topic_callback(const can_msgs::Frame::ConstPtr& msg);
    private:
};
class Dropper: public Peripherals{
    public:
        Dropper(ros::NodeHandle* n_auvic);
        ~Dropper();
        void topic_callback(const can_msgs::Frame::ConstPtr& msg);
    private:
};
class Dvl: public Peripherals{
    public:
        Dvl(ros::NodeHandle* n_auvic);
        ~Dvl();
        void topic_callback(const can_msgs::Frame::ConstPtr& msg);
    private:
};
class Torpedo: public Peripherals{
    public:
        Torpedo(ros::NodeHandle* n_auvic);
        ~Torpedo();
        void topic_callback(const can_msgs::Frame::ConstPtr& msg);
    private:
};
class Hydrophone: public Peripherals{
    public:
        Hydrophone(ros::NodeHandle* n_auvic);
        ~Hydrophone();
        void topic_callback(const can_msgs::Frame::ConstPtr& msg);
    private:
};
class Lcd_Board: public Peripherals{
    public:
        Lcd_Board(ros::NodeHandle* n_auvic);
        ~Lcd_Board();
        void topic_callback(const can_msgs::Frame::ConstPtr& msg);
    private:
};

#endif /* PERIPHERAL_MANAGER_HPP */