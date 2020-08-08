// auvic libraries
#include <peripheral_manager.hpp>

// @param1 nodehandle to speak with auvic topics
Powerboard::Powerboard(ros::NodeHandle & n_auvic){
    PB_deviceName = n_auvic.subscribe<can_msgs::Frame>("protocol_MID_PB_deviceName", 1, &Powerboard::get_PB_active, this);
    PB_envData = n_auvic.subscribe<can_msgs::Frame>("protocol_MID_PB_envData", 1, &Powerboard::get_PB_envData, this);
    PB_battVoltages = n_auvic.subscribe<can_msgs::Frame>("protocol_MID_PB_battVoltages", 1, &Powerboard::get_PB_battVoltages, this);
    PB_battCurrents = n_auvic.subscribe<can_msgs::Frame>("protocol_MID_PB_battCurrents", 1, &Powerboard::get_PB_battCurrents, this);
}

Powerboard::~Powerboard(){};

void Powerboard::get_PB_active(const can_msgs::Frame::ConstPtr& msg){
    Peripherals::device_list.at(devices::Powerboard) = Peripherals::device_status::online;
    if( Peripherals::device_list.at(devices::Powerboard) == Peripherals::device_status::online){
        ROS_INFO_ONCE("DEVICES/POWERBOARD_NODE: Power Board is active!");
    }
    else {
        ROS_WARN_ONCE("DEVICES/POWERBOARD_NODE: Power Board is not active");
    }
}
void Powerboard::get_PB_envData(const can_msgs::Frame::ConstPtr& msg){

}
void Powerboard::get_PB_battVoltages(const can_msgs::Frame::ConstPtr& msg){

}
void Powerboard::get_PB_battCurrents(const can_msgs::Frame::ConstPtr& msg){
    
}
