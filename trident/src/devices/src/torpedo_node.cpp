// auvic libraries
#include <peripheral_manager.hpp>

int main(int argc, char ** argv) {
    // Setup ROS stuff
    ros::init(argc, argv, "Devices_torpedo_node");
    ros::NodeHandle n_auvic("protocols");
    // start torpedo node
    Torpedo torpedo(n_auvic);
    ros::spin();
    return 0;
}
    