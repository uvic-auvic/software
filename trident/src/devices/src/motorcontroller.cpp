// auvic libraries
#include <peripheral_manager.hpp>

// @param1 nodehandle to speak with auvic topics
Motorcontroller::Motorcontroller(ros::NodeHandle* n_auvic){
        this->sub = n_auvic->subscribe<can_msgs::Frame>("motors", 10, &Motorcontroller::topic_callback, this);
        this->client = n_auvic->serviceClient<monitor::GetDeviceMessage>("toCAN");
}

Motorcontroller::~Motorcontroller(){};


// TODO: what to do when reading from the topic
// Instead of polling, maybe use a timer and interrupt it. see ROS timers for details
void Motorcontroller::topic_callback(const can_msgs::Frame::ConstPtr& msg) {
    ROS_INFO_ONCE("Motorcontroller: message received");
}