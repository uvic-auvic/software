// auvic libraries
#include <peripheral_manager.hpp>

// @param1 nodehandle to speak with auvic topics
Lcd_Board::Lcd_Board(ros::NodeHandle & n_auvic){
}

Lcd_Board::~Lcd_Board(){};

void Lcd_Board::get_LCD_active(const can_msgs::Frame::ConstPtr& msg){
    ROS_INFO("Lcd_Board is active!");
    Peripherals::device_list.at(devices::Lcd_Board) = Peripherals::device_status::online;
    if( Peripherals::device_list.at(devices::Lcd_Board) == Peripherals::device_status::online){
        ROS_INFO_ONCE("DEVICES/Lcd_Board_node: Lcd_Board is active!");
    }
    else {
        ROS_WARN_ONCE("DEVICES/Lcd_Board_node: Lcd_Board is not active");
    }
}