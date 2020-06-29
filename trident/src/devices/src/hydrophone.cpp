// auvic libraries
#include <peripheral_manager.hpp>

// @param1 nodehandle to speak with auvic topics
Hydrophone::Hydrophone(ros::NodeHandle* n_auvic){
        this->sub = n_auvic->subscribe<can_msgs::Frame>("acoustics", 10, &Hydrophone::topic_callback, this);
        this->client = n_auvic->serviceClient<auvic_msgs::devices_to_monitor>("toCAN");
}

Hydrophone::~Hydrophone(){};


// TODO: what to do when reading from the topic
// Instead of polling, maybe use a timer and interrupt it. see ROS timers for details
void Hydrophone::topic_callback(const can_msgs::Frame::ConstPtr& msg) {
    ROS_INFO_ONCE("hydrophone: message received");
}