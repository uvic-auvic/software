#include "can_manager.hpp"

HardwareToSoftware::HardwareToSoftware(ros::NodeHandle & n, ros::NodeHandle & nh) {
    Get_The_Message_From_The_World = nh.subscribe<can_msgs::Frame>("received_messages", 1000, &HardwareToSoftware::World_To_Jetson, this);
    MC_deviceName = n.advertise<can_msgs::Frame>("protocol_MID_MC_deviceName", 1);
    MC_motorRPMLow = n.advertise<can_msgs::Frame>("protocol_MID_MC_motorRPMLow", 1);
    MC_motorRPMHigh = n.advertise<can_msgs::Frame>("protocol_MID_MC_motorRPMHigh", 1);
    MC_ISOTP = n.advertise<can_msgs::Frame>("protocol_MID_MC_ISOTP", 1);
    PB_deviceName = n.advertise<can_msgs::Frame>("protocol_MID_PB_deviceName", 1);
    PB_envData = n.advertise<can_msgs::Frame>("protocol_MID_PB_envData", 1);
    PB_battVoltages = n.advertise<can_msgs::Frame>("protocol_MID_PB_battVoltages", 1);
    PB_battCurrents = n.advertise<can_msgs::Frame>("protocol_MID_PB_battCurrents", 1);
}

// Callback from the CAN bus
void HardwareToSoftware::World_To_Jetson(const can_msgs::Frame::ConstPtr& msg){
    // Messages sent by canSEND must be in hex. so 21U means 0x15 and you'd send 015# 
    switch(msg->id){
        case 0x15: //protocol_MID_MC_deviceName (21U)
            ROS_INFO_ONCE("MONITOR/HARDWARETOSOFTWARE: Message [%d] made it in. Next lets see if devices/motorcontroller can read it.", msg->id);
            MC_deviceName.publish(msg);
            break;
        case 0x16: //protocol_MID_MC_motorRPMLow
            ROS_INFO_ONCE("MONITOR/HARDWARETOSOFTWARE: Message made it.");
            MC_motorRPMLow.publish(msg);
            break;
        case 0x17: //protocol_MID_MC_motorRPMHigh
            ROS_INFO_ONCE("MONITOR/HARDWARETOSOFTWARE: Message made it.");
            MC_motorRPMHigh.publish(msg);
            break;
        case 0x18: //protocol_MID_MC_ISOTP
            ROS_INFO_ONCE("MONITOR/HARDWARETOSOFTWARE: Message made it.");
            MC_ISOTP.publish(msg);
            break;
        case 0x29: //protocol_MID_PB_deviceName (41U)
            ROS_INFO_ONCE("MONITOR/HARDWARETOSOFTWARE: Message made it.");
            PB_deviceName.publish(msg);
            break;
        case 0x30: //protocol_MID_PB_envData,
            ROS_INFO_ONCE("MONITOR/HARDWARETOSOFTWARE: Message made it.");
            PB_envData.publish(msg);
            break;
        case 0x31: //protocol_MID_PB_battVoltages
            ROS_INFO_ONCE("MONITOR/HARDWARETOSOFTWARE: Message made it.");
            PB_battVoltages.publish(msg);
            break;
        case 0x32: //protocol_MID_PB_battCurrents
            ROS_INFO_ONCE("MONITOR/HARDWARETOSOFTWARE: Message made it.");
            PB_battCurrents.publish(msg);
            break;
        default:
            ROS_ERROR("MONITOR/HARDWARETOSOFTWARE: BAD STATE WITH [%d]", msg->id);
            break;
    }
}; // end 'World_To_Jetson'