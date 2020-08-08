// auvic libraries
#include <peripheral_manager.hpp>

int main(int argc, char ** argv) {
    // Setup ROS stuff
    ros::init(argc, argv, "Devices_motorcontroller_node");
    ros::NodeHandle n_auvic("protocols");
    // start motorcontroller node
    Motorcontroller motorcontroller(n_auvic);
    ros::spin();
    return 0;
}
    