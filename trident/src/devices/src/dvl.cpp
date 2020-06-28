// auvic libraries
#include <peripheral_manager.hpp>

// @param1 nodehandle to speak with auvic topics
Dvl::Dvl(ros::NodeHandle* n_auvic){
        this->sub = n_auvic->subscribe<can_msgs::Frame>("tracker", 10, &Dvl::topic_callback, this);
        this->client = n_auvic->serviceClient<monitor::GetDeviceMessage>("toCAN");
}

Dvl::~Dvl(){};


// TODO: what to do when reading from the topic
// Instead of polling, maybe use a timer and interrupt it. see ROS timers for details
void Dvl::topic_callback(const can_msgs::Frame::ConstPtr& msg) {
    ROS_INFO_ONCE("dvl: message received");
}