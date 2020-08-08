#include "can_manager.hpp"

/* SoftwareToHardware.CPP
    This node subscribes to several message topics and will publish to the "sent_messages" topic.
    TODO: After confirming this works on the submarine, switch all output messages to ROS_ERROR.
*/

int main(int argc, char ** argv) {
    // Setup ROS stuff
    ros::init(argc, argv, "Monitor_SoftwareToHardware_node");
    // "protocols" is a namespace. This namespace is used to subscribe to the messages
    // "" is used to access the socket_bridge topic 'received_messages'
    ros::NodeHandle n("protocols"), nh("");
    SoftwareToHardware Can_Manager_Software_to_Hardware(n, nh);
    ros::spin();
    return 0;
}
    