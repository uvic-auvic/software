// auvic libraries
#include <peripheral_manager.hpp>

// @param1 nodehandle to speak with auvic topics
Grabber::Grabber(ros::NodeHandle & n_auvic){
}

Grabber::~Grabber(){};

void Grabber::get_GRA_active(const can_msgs::Frame::ConstPtr& msg){
    ROS_INFO("Grabber is active!");
    Peripherals::device_list.at(devices::Grabber) = Peripherals::device_status::online;
    if( Peripherals::device_list.at(devices::Grabber) == Peripherals::device_status::online){
        ROS_INFO_ONCE("DEVICES/Grabber_node: Grabber is active!");
    }
    else {
        ROS_WARN_ONCE("DEVICES/Grabber_node: Grabber is not active");
    }
}