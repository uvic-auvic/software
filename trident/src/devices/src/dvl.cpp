// auvic libraries
#include <peripheral_manager.hpp>

// @param1 nodehandle to speak with auvic topics
Dvl::Dvl(ros::NodeHandle & n_auvic){}

Dvl::~Dvl(){};

void Dvl::get_DVL_active(const can_msgs::Frame::ConstPtr& msg){
    ROS_INFO("DVL is active!");
    Peripherals::device_list.at(devices::Dvl) = Peripherals::device_status::online;
    if( Peripherals::device_list.at(devices::Dvl) == Peripherals::device_status::online){
        ROS_INFO_ONCE("DEVICES/DVL_node: DVL is active!");
    }
    else {
        ROS_WARN_ONCE("DEVICES/DVL_node: DVL is not active");
    }
}