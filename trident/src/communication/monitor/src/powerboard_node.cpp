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
bool Powerboard::send_frame(const can_msgs::Frame::ConstPtr& msg){
    // add a node label
    this->srv.request.node_name = "powerboard";
    // add specific message
    // this->srv.request.msg = @param1;
    if(this->client.call(this->srv)){
        ROS_INFO("From monitor/can_manager: %s", this->srv.response.ack.c_str());
    } else {
        ROS_ERROR("Failed to call service 'toCAN' from Peripherals/powerboard_node");
        return false;
    }
    ROS_INFO("message sent.");
    return true;
}

int main(int argc, char ** argv) {
    // Setup ROS stuff
    ros::init(argc, argv, "powerboard_node");
    // "~" is a private namespace. We use it to connect with our own topics.
    ros::NodeHandle n_auvic("peripherals");
    // start powerboard node
    Powerboard powerboard(&n_auvic);

    ros::spin();
    return 0;
}
    