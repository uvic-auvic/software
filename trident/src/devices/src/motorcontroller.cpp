// auvic libraries
#include <peripheral_manager.hpp>

// @param1 nodehandle to speak with auvic topics
Motorcontroller::Motorcontroller(ros::NodeHandle & n_auvic) {
    MC_deviceName = n_auvic.subscribe<can_msgs::Frame>("protocol_MID_MC_deviceName", 1, &Motorcontroller::get_MC_active, this);
    MC_motorRPMLow = n_auvic.subscribe<can_msgs::Frame>("protocol_MID_MC_motorRPMLow", 1, &Motorcontroller::get_MC_motorRPMLow, this);
    MC_motorRPMHigh = n_auvic.subscribe<can_msgs::Frame>("protocol_MID_MC_motorRPMHigh", 1, &Motorcontroller::get_MC_motorRPMHigh, this);
    MC_ISOTP = n_auvic.subscribe<can_msgs::Frame>("protocol_MID_MC_ISOTP", 1, &Motorcontroller::get_MC_ISOTP, this);
}

Motorcontroller::~Motorcontroller(){};

void Motorcontroller::get_MC_active(const can_msgs::Frame::ConstPtr& msg){
    ROS_INFO("Motor Controller is active!");
    Peripherals::device_list.at(devices::Motorcontroller) = Peripherals::device_status::online;
    if( Peripherals::device_list.at(devices::Motorcontroller) == Peripherals::device_status::online){
        ROS_INFO_ONCE("DEVICES/MOTORCONTROLLER_NODE: motor controller is active!");
    }
    else {
        ROS_WARN_ONCE("DEVICES/MOTORCONTROLLER_NODE: motor controller is not active");
    }
}
void Motorcontroller::get_MC_motorRPMLow(const can_msgs::Frame::ConstPtr& msg){

}
void Motorcontroller::get_MC_motorRPMHigh(const can_msgs::Frame::ConstPtr& msg){

}
void Motorcontroller::get_MC_ISOTP(const can_msgs::Frame::ConstPtr& msg){
    
}
