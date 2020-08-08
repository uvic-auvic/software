#include "can_manager.hpp"

SoftwareToHardware::SoftwareToHardware(ros::NodeHandle & n, ros::NodeHandle & nh) {
    // This will publish to the socketcan topic
    Send_The_Message_To_The_World = nh.advertise<can_msgs::Frame>("sent_messages", 10);

    // Intraproccess Publishing: For when pubs and subs are in a single node.
    TRIDENT_deviceName = n.subscribe<auvic_msgs::Monitor_Jetson_To_World>("protocol_MID_TRIDENT_deviceName", 10, &SoftwareToHardware::Jetson_To_World, this);
    TRIDENT_motorSetSpeed = n.subscribe<auvic_msgs::Monitor_Jetson_To_World>("protocol_MID_TRIDENT_motorSetSpeed", 10, &SoftwareToHardware::Jetson_To_World, this);
    TRIDENT_powerEnable = n.subscribe<auvic_msgs::Monitor_Jetson_To_World>("protocol_MID_TRIDENT_powerEnable", 10, &SoftwareToHardware::Jetson_To_World, this);
    TRIDENT_MCISOTP = n.subscribe<auvic_msgs::Monitor_Jetson_To_World>("protocol_MID_TRIDENT_MCISOTP", 10, &SoftwareToHardware::Jetson_To_World, this);
}

// Handles messages
void SoftwareToHardware::Jetson_To_World(const auvic_msgs::Monitor_Jetson_To_World::ConstPtr& msg){
    ROS_DEBUG("MONITOR/SOFTWARETOHARDWARE: Sending '[%s]' msg...\n", msg->message_name.data.c_str());
    can_msgs::Frame output;
    // 11 bit max
    output.id = (uint32_t)msg->CAN_MSG.id;
    // is this a remote frame (true) or a data frame (false)
    output.is_rtr = msg->CAN_MSG.is_rtr;
    // does the message contain more then 64 bits?
    output.is_extended = msg->CAN_MSG.is_extended;
    // Is this frame is an error message?
    output.is_error = msg->CAN_MSG.is_error;
    // data length code: 1 byte to 8 byte [1..8]
    output.dlc = msg->CAN_MSG.dlc;
    // Data is outputed big endian first
    for(int i = 0; i < output.dlc; i++){
        output.data[i] = msg->CAN_MSG.data[i];
    }
    Send_The_Message_To_The_World.publish(output);
    ROS_DEBUG("MONITOR/SOFTWARETOHARDWARE: Sent '[%s]' msg.\n", msg->message_name.data.c_str());
    
}; // end 'msg_deviceName'