// auvic libraries
#include <peripheral_manager.hpp>

// @param1 nodehandle to speak with auvic topics
Imu::Imu(ros::NodeHandle & n_auvic){
}

Imu::~Imu(){};

void Imu::get_IMU_active(const can_msgs::Frame::ConstPtr& msg){
    ROS_INFO("Imu is active!");
    Peripherals::device_list.at(devices::Imu) = Peripherals::device_status::online;
    if( Peripherals::device_list.at(devices::Imu) == Peripherals::device_status::online){
        ROS_INFO_ONCE("DEVICES/Imu_node: Imu is active!");
    }
    else {
        ROS_WARN_ONCE("DEVICES/Imu_node: Imu is not active");
    }
}
/// TODO: Send Message ID and data to send_frame
void get_temperature(const can_msgs::Frame::ConstPtr& msg){};
/// TODO: Send Message ID and data to send_frame
void get_euler_stable(const can_msgs::Frame::ConstPtr& msg){};
/// TODO: Send Message ID and data to send_frame
void get_mag_accel_gyro_stable(const can_msgs::Frame::ConstPtr& msg){};
/// TODO: Send Message ID and data to send_frame
void get_mag_accel_gyro(const can_msgs::Frame::ConstPtr& msg){};
/// TODO: Send Message ID and data to send_frame
void get_velocity(const can_msgs::Frame::ConstPtr& msg){};
/// TODO: Send Message ID and data to send_frame
void set_velocity(const can_msgs::Frame::ConstPtr& msg){};
/// TODO: Send Message ID and data to send_frame
void update_velocity(const can_msgs::Frame::ConstPtr& msg){};
/// TODO: do a CHECKSUM test
void verify_response(){};