// auvic libraries
#include <peripheral_manager.hpp>

// @param1 nodehandle to speak with auvic topics
Lcd_Board::Lcd_Board(ros::NodeHandle* n_auvic){
        this->sub = n_auvic->subscribe<can_msgs::Frame>("RGBdebug", 10, &Lcd_Board::topic_callback, this);
        this->client = n_auvic->serviceClient<monitor::GetCanMSG>("toCAN");
}

Lcd_Board::~Lcd_Board(){};


// TODO: what to do when reading from the topic
// Instead of polling, maybe use a timer and interrupt it. see ROS timers for details
void Lcd_Board::topic_callback(const can_msgs::Frame::ConstPtr& msg) {
    ROS_INFO_ONCE("message received");
}