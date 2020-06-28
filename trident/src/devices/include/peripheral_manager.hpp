#ifndef PERIPHERAL_MANAGER_HPP
#define PERIPHERAL_MANAGER_HPP
//C++ library
#include <memory>

// ROS kernal
#include <ros/ros.h>
#include <ros/console.h>
#include "can_msgs/Frame.h"

//auvic library
#include "monitor/GetDeviceMessage.h"
#include "devices/rpms.h"
#include "devices/powerboard.h"
#include "devices/polar_num.h"
#include "devices/orientation.h"
#include "devices/motor_enums.h"
#include "devices/imu.h"
#include "devices/hydro_fft.h"
#include "devices/hydro.h"
#include "devices/depth.h"

#include "devices/avg_data.h"
#include "devices/power_enable.h"

class Peripherals{
    public:
        Peripherals (){};
        ~Peripherals (){};
        void topic_callback(const can_msgs::Frame::ConstPtr& msg);
        // TODO: construct can_msgs frame from data
        bool send_frame(const can_msgs::Frame::ConstPtr& data);
    
        // peripheral is turned off by default
        bool ignore = true;
        monitor::GetDeviceMessage srv;
        can_msgs::Frame msg;
        uint8_t data[8];
        ros::Subscriber sub;
        ros::ServiceClient client;
};

class Powerboard: public Peripherals{
    public:
        // using powerboardInfo = devices::powerboard;
        // using powerEnableReq = devices::power_enable::Request;
        // using powerEnableRes = devices::power_enable::Response;
        // using AvgDataReq = devices::avg_data::Request;
        // using AvgDataRes = devices::avg_data::Response;

        Powerboard(ros::NodeHandle* n_auvic);
        ~Powerboard();
        void topic_callback(const can_msgs::Frame::ConstPtr& msg);
        // TODO: get_powerboard_data: will create a can_msgs file and pass to 'send_frame'
        bool get_powerboard_data();
        // TODO: power_enabler: see polaris for implementation
        bool power_enabler();
        // TODO: average_ext_pressure: see polaris for implementation
        bool average_ext_pressure();

    private:
        std::string PB_deviceName, PB_envData, PB_battVoltages, PB_battCurrents;
        
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