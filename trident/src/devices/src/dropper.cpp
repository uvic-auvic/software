// auvic libraries
#include <peripheral_manager.hpp>

// @param1 nodehandle to speak with auvic topics
Dropper::Dropper(ros::NodeHandle & n_auvic){
}

Dropper::~Dropper(){};

void Dropper::get_DRO_active(const can_msgs::Frame::ConstPtr& msg){
    ROS_INFO("Dropper is active!");
    Peripherals::device_list.at(devices::Dropper) = Peripherals::device_status::online;
    if( Peripherals::device_list.at(devices::Dropper) == Peripherals::device_status::online){
        ROS_INFO_ONCE("DEVICES/Dropper_node: Dropper is active!");
    }
    else {
        ROS_WARN_ONCE("DEVICES/Dropper_node: Dropper is not active");
    }
}