// auvic libraries
#include <peripheral_manager.hpp>

// @param1 nodehandle to speak with auvic topics
Dropper::Dropper(ros::NodeHandle* n_auvic){
        this->sub = n_auvic->subscribe<can_msgs::Frame>("SRweapon", 10, &Dropper::topic_callback, this);
        this->client = n_auvic->serviceClient<monitor::GetDeviceMessage>("toCAN");
}

Dropper::~Dropper(){};


// TODO: what to do when reading from the topic
// Instead of polling, maybe use a timer and interrupt it. see ROS timers for details
void Dropper::topic_callback(const can_msgs::Frame::ConstPtr& msg) {
    ROS_INFO_ONCE("dropper: message received");
}