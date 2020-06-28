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


// handles messages received from the can bus
// @param: msg: a ptr to the message received from the can bus. 
void Can_Manager::From_Can(const can_msgs::Frame::ConstPtr& msg){
    ROS_INFO_ONCE("The frame id is: [%d]", msg->id);
    
    // lookup topic from id list and send message
    uint32_t numOfSubs1, numOfSubs2, numOfSubs3, numOfSubs4, numOfSubs5, numOfSubs6, numOfSubs7, numOfSubs8, numOfSubs9;
    switch(msg->id){
        case 1: // imu
            ROS_INFO_ONCE("publishing to 'inertia' topic from imu.");
            //wait until there is subscribers on the topic
            numOfSubs1 = this->Peripheral_imu.getNumSubscribers();
            while(numOfSubs1 == 0){
                numOfSubs1 = this->Peripheral_imu.getNumSubscribers();
                ROS_ERROR_ONCE("monitor/can_manager_node: imu is not on the correct topic");
            }
            this->Peripheral_imu.publish(msg);
            ROS_INFO_ONCE("Published message to 'inertia'.");
            break;
        case 2: // powerboard
            ROS_INFO_ONCE("publishing to 'power' topic from powerboard.");
            //wait until there is subscribers on the topic
            numOfSubs2 = this->Peripheral_powerboard.getNumSubscribers();
            while(numOfSubs2 == 0){
                numOfSubs2 = this->Peripheral_powerboard.getNumSubscribers();
                ROS_ERROR_ONCE("monitor/can_manager_node: powerboard is not on the correct topic");
            }
            this->Peripheral_powerboard.publish(msg);
            ROS_INFO_ONCE("Published message to 'power'.");
            break;
        case 3: // hydrophone
            ROS_INFO_ONCE("publishing to 'acoustics' topic from hydrophone.");
            //wait until there is subscribers on the topic
            numOfSubs3 = this->Peripheral_hydrophone.getNumSubscribers();
            while(numOfSubs3 == 0){
                numOfSubs3 = this->Peripheral_hydrophone.getNumSubscribers();
                ROS_ERROR_ONCE("monitor/can_manager_node: hydrophone is not on the correct topic");
            }
            this->Peripheral_hydrophone.publish(msg);
            ROS_INFO_ONCE("Published message to 'acoustic'.");
            break;
        case 4: // motorcontroller
            ROS_INFO_ONCE("publishing to 'motors' topic from motorcontroller.");
            //wait until there is subscribers on the topic
            numOfSubs4 = this->Peripheral_motorcontroller.getNumSubscribers();
            while(numOfSubs4 == 0){
                numOfSubs4 = this->Peripheral_motorcontroller.getNumSubscribers();
                ROS_ERROR_ONCE("monitor/can_manager_node: motorcontroller is not on the correct topic");
            }
            this->Peripheral_motorcontroller.publish(msg);
            ROS_INFO_ONCE("Published message to 'motors'.");
            break;
        case 5: // torpedo
            ROS_INFO_ONCE("publishing to 'LRweapon' topic from torpedo.");
            //wait until there is subscribers on the topic
            numOfSubs5 = this->Peripheral_torpedo.getNumSubscribers();
            while(numOfSubs5 == 0){
                numOfSubs5 = this->Peripheral_torpedo.getNumSubscribers();
                ROS_ERROR_ONCE("monitor/can_manager_node: torpedo is not on the correct topic");
            }
            this->Peripheral_torpedo.publish(msg);
            ROS_INFO_ONCE("Published message to 'LRweapon'.");
            break;
        case 6: // lcd_board
            ROS_INFO_ONCE("publishing to 'RGBdebug' topic from lcd_board.");
            //wait until there is subscribers on the topic
            numOfSubs6 = this->Peripheral_lcd_board.getNumSubscribers();
            while(numOfSubs6 == 0){
                ROS_ERROR_ONCE("monitor/can_manager_node: lcd_board is not on the correct topic");
                numOfSubs6 = this->Peripheral_lcd_board.getNumSubscribers();
            }
            this->Peripheral_lcd_board.publish(msg);
            ROS_INFO_ONCE("Published message to 'RGBdebug'.");
            break;
        case 7: // ball_dropper
            ROS_INFO_ONCE("publishing to 'SRweapon' topic from Ball_dropper.");
            //wait until there is subscribers on the topic
            numOfSubs7 = this->Peripheral_dropper.getNumSubscribers();
            while(numOfSubs7 == 0){
                ROS_ERROR_ONCE("monitor/can_manager_node: dropper is not on the correct topic");
                numOfSubs7 = this->Peripheral_dropper.getNumSubscribers();
            }
            this->Peripheral_dropper.publish(msg);
            ROS_INFO_ONCE("Published message to 'SRweapon'.");
            break;
        case 8: // grabber
            ROS_INFO_ONCE("publishing to 'limb' topic from Grabber.");
            //wait until there is subscribers on the topic
            numOfSubs8 = this->Peripheral_imu.getNumSubscribers();
            while(numOfSubs8 == 0){
                ROS_ERROR_ONCE("monitor/can_manager_node: grabber is not on the correct topic");
                numOfSubs8 = this->Peripheral_imu.getNumSubscribers();
            }
            this->Peripheral_imu.publish(msg);
            ROS_INFO_ONCE("Published message to 'limb'.");
            break;
        case 9: // dvl
            ROS_INFO_ONCE("publishing to 'tracker' topic from dvl.");
            //wait until there is subscribers on the topic
            numOfSubs9 = this->Peripheral_dvl.getNumSubscribers();
            while(numOfSubs9 == 0){
                ROS_ERROR_ONCE("monitor/can_manager_node: dvl is not on the correct topic");
                numOfSubs9 = this->Peripheral_dvl.getNumSubscribers();
            }
            this->Peripheral_dvl.publish(msg);
            ROS_INFO_ONCE("Published message to 'tracker'.");
            break;
        default:
            ROS_DEBUG_ONCE("msg_id from 'THECALLfromTheBUS' in Monitor package was incorrect");
            break;
    }
    // continue
}  

//  publishes message to socketCAN
bool Can_Manager::To_Can(GetDeviceMessageReq& req, GetDeviceMessageRes& res){
    ROS_INFO_ONCE("Sending msg from [%s] to [socketcan_bridge].\n", req.node_name.c_str());
    this->send_.publish(req.msg);
    ROS_INFO_ONCE("Published message.\n");
    res.ack = "sent.";
    return true;
}

// TODO: Assemble can_msg for socketcan_bridge
can_msgs::Frame Can_Manager::Make_Can_msg(){
    can_msgs::Frame msg;
    return msg;
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