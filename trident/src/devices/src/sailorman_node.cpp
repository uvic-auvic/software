// auvic libraries
#include <sailorman.hpp>

/*
    SailorMan takes a service request to get data inside the 'Peripherals' class.
    It publishes data through the 'Protocols' topics.

    BE REALLY CAREFULL ABOUT NAMESPACES. YOUR FIRST ISSUE WILL PROBABLY ORGINATE FROM TRYING TO CONNECT TO THE SERVER BUT NOT INCLUDING THE CORRECT NS.
*/

int main(int argc, char ** argv) {
    // Setup ROS stuff
    ros::init(argc, argv, "sailorman_node");
    ros::NodeHandle n_auvic("protocols"), server("");
    // sailorman
    SailorMan Jack_Sparrow(n_auvic, server);
    ros::spin();
    return 0;
}
    