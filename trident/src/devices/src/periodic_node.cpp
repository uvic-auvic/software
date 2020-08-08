// auvic libraries
#include <sailorman.hpp>

/*
this node periodically publishes a list of data. Add the messages you want to be sent onto the bus here and
push it back into the topic vector in the constructor.
*/

int main(int argc, char ** argv) {
    // Setup ROS stuff
    ros::init(argc, argv, "periodic_node");
    ros::NodeHandle n_auvic("protocols");
    // sailorman
    SailorMan Captain_Hook(n_auvic);
    
    // update CAN bus with jetson data every 5 seconds
    ros::Rate loop_rate(0.2);
    while(ros::ok()){
        // Currently, this only sends a heartbeat into the bus, 
        for(int i = 0 ; i < Captain_Hook.list_of_publishing_topics.size(); i++){
            if( Captain_Hook.list_of_publishing_topics.at(i).compare("protocol_MID_TRIDENT_deviceName") == 0){
                ROS_INFO("Devices/sailorman_node: Sending Jetson heartbeat to CAN bus.");
                auvic_msgs::Monitor_Jetson_To_World packet;
                packet.message_name.data = "protocol_MID_TRIDENT_deviceName";
                packet.CAN_MSG.id = 1;
                packet.CAN_MSG.dlc = 1;
                packet.CAN_MSG.is_extended = false;
                packet.CAN_MSG.is_error = false;
                packet.CAN_MSG.is_rtr = false;
                packet.CAN_MSG.data[0] = Peripherals::get_Jetson_status()? 1: 0;
                Captain_Hook.TRIDENT_deviceName.publish(packet);
            }
            // add other topics that will be sent periodically below
        };
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
    