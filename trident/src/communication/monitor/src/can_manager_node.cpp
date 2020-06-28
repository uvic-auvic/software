// auvic library
#include <can_manager.hpp>

/* CAN_MANAGER_NODE.CPP
    this node functions as our message arbitratior to the socketcan_bridge package. The nodes in the 
    'devices' package are the only ones that can send service requests on "toCan." all application methods 
    are created in the devices package.

    TODO: After confirming this works on the submarine. Switch all output messages to ROS_ERROR for easier access.
*/

int main(int argc, char ** argv) {
    // Setup ROS stuff
    ros::init(argc, argv, "can_manager_node");
    // "peripheral" is a namespace. This namespace is used to publish to the peripheral topics
    // "" is used to access the socket_bridge topics 'sent_messages' and 'received_messages'
    ros::NodeHandle n("peripherals"), nh("");
    
  
    // can_manager begins
    Can_Manager can_manager(&n, &nh);
    // TODO: Check what devices are on the bus
    //can_manager.startup_routine(&nh);

    ros::spin();
    return 0;
}
    