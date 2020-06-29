// ROS kernal
#include <ros/ros.h>
// auvic libraries
#include "can_manager.hpp"

// @param1 nodehandle to speak with auvic topics
// @param2 nodehandle reserved for socketcan topics
Can_Manager::Can_Manager(ros::NodeHandle* n_auvic, ros::NodeHandle* n_socketcan){

        // Socket topics
        this->send_ = n_socketcan->advertise<can_msgs::Frame>("sent_messages", 10);
        this->receive_ = n_socketcan->subscribe<can_msgs::Frame>("received_messages", 10, &Can_Manager::From_Can, this);
        
        // Services
        this->node_server_ = n_auvic->advertiseService("toCAN", &Can_Manager::To_Can, this);

        // Peripheral topics
        this->Peripheral_torpedo = n_auvic->advertise<can_msgs::Frame>("LRweapon", 10);
        this->Peripheral_powerboard = n_auvic->advertise<can_msgs::Frame>("power", 10);
        this->Peripheral_motorcontroller = n_auvic->advertise<can_msgs::Frame>("motors", 10);
        this->Peripheral_lcd_board = n_auvic->advertise<can_msgs::Frame>("RGBdebug", 10);
        this->Peripheral_dvl = n_auvic->advertise<can_msgs::Frame>("tracker", 10);
        this->Peripheral_imu = n_auvic->advertise<can_msgs::Frame>("inertia", 10);
        this->Peripheral_grabber = n_auvic->advertise<can_msgs::Frame>("limb", 10); 
        this->Peripheral_dropper = n_auvic->advertise<can_msgs::Frame>("SRweapon", 10);
        this->Peripheral_hydrophone = n_auvic->advertise<can_msgs::Frame>("acoustics", 10);   
}

void Can_Manager::message_handle(ros::Publisher Peripheral_topic_handle, std::string topic, can_msgs::Frame msg){
    ROS_INFO_ONCE("can_manager: publishing to [%s] topic.\n", topic.c_str());
    // Ignore message if the subscriber is not active
    uint32_t subCount = Peripheral_topic_handle.getNumSubscribers();
    if(subCount == 0){
        subCount = Peripheral_topic_handle.getNumSubscribers();
        ROS_ERROR_ONCE("can_manager: not enough subscribers. Throwing away message...\n");
    } else {
        Peripheral_topic_handle.publish(msg);
        ROS_INFO_ONCE("Published message to '[%s]'.\n", topic.c_str());
    }
}

// handles messages received from the can bus
// @param: msg: a ptr to the message received from the can bus. 
void Can_Manager::From_Can(const can_msgs::Frame::ConstPtr& msg){
    ROS_INFO_ONCE("can_manager: The frame id is [%d].\n", msg->id);
    // get message id
    protocol_MID_E messageID = static_cast<protocol_MID_E>(msg->id);

    // lookup the message
    /// add messages by modifying protocol.hpp 
    switch(messageID){
        case protocol_MID_MC_deviceName:
            this->message_handle(this->Peripheral_motorcontroller, "motors", *msg);
            break;
        case protocol_MID_MC_motorRPMLow:
            this->message_handle(this->Peripheral_motorcontroller, "motors", *msg);
            break;
        case protocol_MID_MC_motorRPMHigh:
            this->message_handle(this->Peripheral_motorcontroller, "motors", *msg);
            break;
        case protocol_MID_PB_deviceName:
            this->message_handle(this->Peripheral_powerboard, "power", *msg);
            break;
        case protocol_MID_PB_envData: // Environmental Data
            this->message_handle(this->Peripheral_powerboard, "power", *msg);
            break;
        case protocol_MID_PB_battVoltages:
            this->message_handle(this->Peripheral_powerboard, "power", *msg);
            break;
        case protocol_MID_PB_battCurrents:
            this->message_handle(this->Peripheral_powerboard, "power", *msg);
            break;
    }
    // end callback
}  

// Service: gets data from node, returns an ack, but first assemble can_msg publishes message to socketCAN
bool Can_Manager::To_Can(GetMonitorReq& req, GetMonitorRes& res){
    // make can_frame
    can_msgs::Frame package;
    Make_Can_msg(req, package);
    // Send onto can bus
    ROS_INFO_ONCE("Sending msg from [%s] to [socketcan_bridge].\n", req.node_name.c_str());
    this->send_.publish(package);
    ROS_INFO_ONCE("can_manager: Published message.\n");
    res.ack = "sent.";
    return true;
}

void Can_Manager::Make_Can_msg(GetMonitorReq& input, can_msgs::Frame& output){
    output.id = (uint32_t)input.message_id.id;
    // remote transmisson request? signfies where the id frame ends for 11 bit frame
    // we always want this as true
    output.is_rtr = true;
    // does the message contain more then 64 bits? keep it false for now and add feature if needed.
    output.is_extended = false;
    // not sure
    output.is_error = false;
    // data length code: set it to max for now, and in the future update, have the services define the frame length.
    /// max = 8 bytes
    output.dlc = 0xFF;
    // ... sigh... okay lets make another list....
    /// frick. I dont want to. - Aman
    output.data = {0x6e, 0x6f, 0x70, 0x65, 0x21, 0x00, 0x00, 0x00};
}

// TODO: check which devices are connected to the CAN bus
void Can_Manager::Startup_routine(ros::NodeHandle* n_auvic){

}

// TODO: Check a single device status on the CAN bus
void Can_Manager::Check_device_status(std::string device_name){

}

// TODO: Check all devices on the CAN bus
void Can_Manager::Check_all_devices_status(){

}

// TODO: do a CRC
void Can_Manager::CRC(){
        
}

