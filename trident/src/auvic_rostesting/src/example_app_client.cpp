#include <ros/ros.h>


int main(int argc, char ** argv) {
    // Setup ROS stuff
    ros::init(argc, argv, "example_app_client");
    ros::NodeHandle n_auvic("protocols");
    // start dropper node
    Dropper dropper(n_auvic);
    ros::spin();
    return 0;
}
    