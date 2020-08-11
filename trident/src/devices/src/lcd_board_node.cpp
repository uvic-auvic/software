// auvic libraries
#include <peripheral_manager.hpp>

int main(int argc, char ** argv) {
    // Setup ROS stuff
    ros::init(argc, argv, "Devices_lcd_board_node");
    ros::NodeHandle n_auvic("protocols");
    // start lcd_board node
    Lcd_Board lcd_board(n_auvic);

    ros::spin();
    return 0;
}
    