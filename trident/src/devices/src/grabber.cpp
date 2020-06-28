// auvic libraries
#include <peripheral_manager.hpp>

// @param1 nodehandle to speak with auvic topics
Grabber::Grabber(ros::NodeHandle* n_auvic){
        this->sub = n_auvic->subscribe<can_msgs::Frame>("limb", 10, &Grabber::topic_callback, this);
        this->client = n_auvic->serviceClient<monitor::GetDeviceMessage>("toCAN");
}

Grabber::~Grabber(){};


// TODO: what to do when reading from the topic
// Instead of polling, maybe use a timer and interrupt it. see ROS timers for details
void Grabber::topic_callback(const can_msgs::Frame::ConstPtr& msg) {
    ROS_INFO_ONCE("grabber: message received");
}