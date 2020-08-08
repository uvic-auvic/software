// auvic libraries
#include <peripheral_manager.hpp>

int main(int argc, char ** argv) {
    // Setup ROS stuff
    ros::init(argc, argv, "grabber_node");
    ros::NodeHandle n_auvic("protocols");
    // start grabber node
    Grabber grabber(n_auvic);

    ros::spin();
    return 0;
}
    