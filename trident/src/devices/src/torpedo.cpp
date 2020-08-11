// auvic libraries
#include <peripheral_manager.hpp>

Torpedo::Torpedo(ros::NodeHandle & n_auvic) {}

Torpedo::~Torpedo(){};

void Torpedo::get_TOR_active(const can_msgs::Frame::ConstPtr& msg){
    ROS_INFO("Torpedo is active!");
    Peripherals::device_list.at(devices::Torpedo) = Peripherals::device_status::online;
    if( Peripherals::device_list.at(devices::Torpedo) == Peripherals::device_status::online){
        ROS_INFO_ONCE("DEVICES/Torpedo_NODE: Torpedo is active!");
    }
    else {
        ROS_WARN_ONCE("DEVICES/Torpedo_NODE: Torpedois not active");
    }
}