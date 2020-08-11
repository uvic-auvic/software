#include "can_manager.hpp"

/* HardwareToSoftware_node.CPP
    This node subscribes to the 'received_messages' topic and will publish to several 'protocol' topic.
    TODO: After confirming this works on the submarine, switch all output messages to ROS_ERROR.
*/

int main(int argc, char ** argv) {
    // Setup ROS stuff
    ros::init(argc, argv, "Monitor_HardwareToSoftware_node");
    // "protocols" is a namespace. This namespace is used to subscribe to the messages
    // "" is used to access the socket_bridge topic 'received_messages'
    ros::NodeHandle n("protocols"), nh("");
    HardwareToSoftware Can_Manager_Hardware_to_Software(n, nh);
    ros::spin();
    return 0;
}
    