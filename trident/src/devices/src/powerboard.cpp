// auvic libraries
#include <peripheral_manager.hpp>

// @param1 nodehandle to speak with auvic topics
Powerboard::Powerboard(ros::NodeHandle* n_auvic){
        this->sub = n_auvic->subscribe<can_msgs::Frame>("power", 10, &Powerboard::topic_callback, this);
        this->client = n_auvic->serviceClient<monitor::GetCanMSG>("toCAN");
}

Powerboard::~Powerboard(){};

// TODO: what to do when reading from the topic
// Instead of polling, maybe use a timer and interrupt it. see ROS timers for details
void Powerboard::topic_callback(const can_msgs::Frame::ConstPtr& msg) {
    ROS_INFO_ONCE("message received");
}

// this method gets called when sending a mesage onto can bus.
// TODO: have methods send data here to be compiled into a proper frame
// bool Powerboard::send_frame(const can_msgs::Frame::ConstPtr& data){
//     // add a node label - just for debugging purposes
//     this->srv.request.node_name = "powerboard";
//     // add the can message to the srv
//     // this->srv.request.msg = ....;
//     if(this->client.call(this->srv)){
//         ROS_INFO("From monitor/can_manager: %s", this->srv.response.ack.c_str());
//     } else {
//         ROS_ERROR("Failed to call service 'toCAN' from Peripherals/powerboard_node");
//         return false;
//     }
//     ROS_INFO("message sent.");
//     return true;
// }

// TODO: get_powerboard_data: will create a can_msgs file and pass to 'send_frame'
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