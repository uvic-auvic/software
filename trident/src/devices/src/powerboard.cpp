// auvic libraries
#include <peripheral_manager.hpp>

// @param1 nodehandle to speak with auvic topics
Powerboard::Powerboard(ros::NodeHandle* n_auvic){
    this->sub = n_auvic->subscribe<can_msgs::Frame>("power", 10, &Powerboard::topic_callback, this);
    this->client = n_auvic->serviceClient<auvic_msgs::devices_to_monitor>("toCAN");
}

Powerboard::~Powerboard(){};

// TODO: what to do when reading from the topic
// Instead of polling, maybe use a timer and interrupt it. see ROS timers for details
void Powerboard::topic_callback(const can_msgs::Frame::ConstPtr& msg) {
    ROS_INFO_ONCE("Powerboard: message received");
}

// TODO: select message ID and initiate the toCan service
bool Powerboard::get_powerboard_data(){
    return false;
}

// TODO: power_enabler: see polaris for implementation
bool Powerboard::power_enabler(){
    return false;
}

// TODO: average_ext_pressure: see polaris for implementation
bool Powerboard::average_ext_pressure(){
    return false;
}

// TODO: return depth data from powerboardData
bool depth_from_sensor(){
    return false;
}

// TODO: return temperature data from powerbardData
bool temperature_from_sensor(){
    return false;
}

