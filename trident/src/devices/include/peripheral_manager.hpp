#ifndef PERIPHERAL_MANAGER_HPP
#define PERIPHERAL_MANAGER_HPP
//C++ library
#include <memory>

// ROS kernal
#include <ros/ros.h>
#include <ros/console.h>
#include "can_msgs/Frame.h"

//auvic library
#include "auvic_msgs/devices_to_monitor.h"

/// powerboard
#include "auvic_msgs/devices_PB_powerboardData.h"
#include "auvic_msgs/devices_PB_avgData.h"
#include "auvic_msgs/devices_PB_powerEnable.h"

/// motorcontroller
#include "auvic_msgs/devices_MC_rpms.h"
#include "auvic_msgs/devices_MC_motorEnums.h"
#include "auvic_msgs/devices_MC_motor.h"
#include "auvic_msgs/devices_MC_motors.h"
#include "auvic_msgs/devices_MC_getMotorEnums.h"


class Peripherals{
    public:
        Peripherals (){};
        ~Peripherals (){};
        void topic_callback(const can_msgs::Frame::ConstPtr& msg);
    
        // peripheral is turned off by default
        /// TODO: add this to the roslaunch file so ROS recognizes the node is disconn.
        bool ignore = true;

        // srvs and msgs
        auvic_msgs::devices_to_monitor srv;
        can_msgs::Frame msg;
        
        // ROS stuff
        ros::Subscriber sub;
        ros::ServiceClient client;
};

class Powerboard: public Peripherals{
    public:
        using powerboardInfo = auvic_msgs::devices_PB_powerboardData;
        using powerEnableReq = auvic_msgs::devices_PB_powerEnable::Request;
        using powerEnableRes = auvic_msgs::devices_PB_powerEnable::Response;
        using AvgDataReq = auvic_msgs::devices_PB_avgData::Request;
        using AvgDataRes = auvic_msgs::devices_PB_avgData::Response;

        Powerboard(ros::NodeHandle* n_auvic);
        ~Powerboard();
        // turn node on
        bool ignore = false;
        /// TODO: what to do when reading from the topic
        void topic_callback(const can_msgs::Frame::ConstPtr& msg);
        /// TODO: select message ID and call send_frame()
        bool get_powerboard_data();
        /// TODO: power_enabler: see polaris for implementation
        bool power_enabler();
        /// TODO: average_ext_pressure: see polaris for implementation
        bool average_ext_pressure();
        /// TODO: return depth data from powerboardData
        bool depth_from_sensor();
        /// TODO: return temperature data from powerbardData
        bool temperature_from_sensor();

    private:
        std::string PB_deviceName, PB_envData, PB_battVoltages, PB_battCurrents;
        
};
class Motorcontroller: public Peripherals{
    public:
        Motorcontroller(ros::NodeHandle* n_auvic);
        ~Motorcontroller();
        /// TODO: what to do when reading from the topic
        void topic_callback(const can_msgs::Frame::ConstPtr& msg);
        /// TODO: Send Message ID and data to send_frame
        bool setMotorPWM();
        /// TODO: Send Message ID and data to send_frame
        bool setAllMotorsPWM();
        /// TODO: Send Message ID and data to send_frame
        bool stoAllMotors();
        /// TODO: Send Message ID and data to send_frame 
        bool getMotorEnums();
    private:
        std::vector<double> pwm_multipliers;
        
};
class Imu: public Peripherals{
    public:
        Imu(ros::NodeHandle* n_auvic);
        ~Imu();
        /// TODO: what to do when reading from the topic
        void topic_callback(const can_msgs::Frame::ConstPtr& msg);
        /// TODO: Send Message ID and data to send_frame
        bool get_temperature();
        /// TODO: Send Message ID and data to send_frame
        bool get_euler_stable();
        /// TODO: Send Message ID and data to send_frame
        bool get_mag_accel_gyro_stable();
        /// TODO: Send Message ID and data to send_frame
        bool get_mag_accel_gyro();
        /// TODO: Send Message ID and data to send_frame
        void get_velocity();
        /// TODO: Send Message ID and data to send_frame
        bool set_velocity();
        /// TODO: Send Message ID and data to send_frame
        void update_velocity();
    private:
        /// TODO: do a CHECKSUM test
        void verify_response();
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
        /// TODO: Send Message ID and data to send_frame
        bool get_raw_data();
        /// TODO: Send Message ID and data to send_frame
        bool get_fft_data();
        /// TODO: Send Message ID and data to send_frame
        bool get_hydro_phases();


    private:
        /// TODO:
        bool acquire_hydro_data();
        /// TODO:
        bool compute_fft();
};
class Lcd_Board: public Peripherals{
    public:
        Lcd_Board(ros::NodeHandle* n_auvic);
        ~Lcd_Board();
        void topic_callback(const can_msgs::Frame::ConstPtr& msg);
    private:
};



#endif /* PERIPHERAL_MANAGER_HPP */