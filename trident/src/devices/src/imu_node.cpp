// auvic libraries
#include <peripheral_manager.hpp>

int main(int argc, char ** argv) {
    // Setup ROS stuff
    ros::init(argc, argv, "Devices_imu_node");
    ros::NodeHandle n_auvic("protocols");
    // start powerboard node
    Imu imu(n_auvic);

    ros::spin();
    return 0;
}
    