// ROS kernal
#include <ros/ros.h>
// auvic library
#include <can_manager.hpp>
// after confirming this works on the submarine. Switch all ROS_DEBUG_ONCE to ROS_ERROR for easier access.

/*
    can_manager will publish all messages to specific topics on the n("~") private namespace.
        -   That means to access them, the node must use a nodehandle of n("~").
        -   Powerboard data will be published to 'power', see robot_model for rest of topics
        -   nodes are not permitted to publish to these topics. Not sure how to enforce this yet.
        -   Before data is sent to a topic, check to see if the subscribers are initialized. This 
            will be done using ros::Publisher::getNumSubscribers() for each topic. Only send data 
            out for the ones that are already set.
    
    
    to send data to the can bus, send a service request to the can_manager using the 'toCAN.srv.'
    The requester must wait until can_manager responds with a 'sent.' to the 'toCAN.src'. 

    I think we should have two lines: one for peripherals and another for motors.

*/

// @param1 vector of devices from yaml
// @param2 nodehandle to speak with auvic topics
// @param3 nodehandle reserved for socketcan topics
Can_Manager::Can_Manager(const std::vector<Device_Property> & properties, ros::NodeHandle* n_auvic){
        // Device list
        this->JSON_Device_List = properties;

        // System topics/services
        this->send_ = n_auvic->advertise<can_msgs::Frame>("sent_messages", 10);
        
        this->node_server_ = n_auvic->advertiseService("toCan", &Can_Manager::toCan, this);
        
        this->receive_ = n_auvic->subscribe<can_msgs::Frame>("received_messages", 10, &Can_Manager::THECALLfromTheBUS, this);
        
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

// @param: msg: a ptr to the message received from the can bus. 
void Can_Manager::THECALLfromTheBUS(const can_msgs::Frame::ConstPtr& msg){
    ROS_INFO_ONCE("The frame id is: [%d]", msg->id);
    std::string topic_name = this->Peripheral_powerboard.getTopic();
    ROS_INFO_ONCE(topic_name.c_str());
    //Peripheral_powerboard.publish(*msg);
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

bool Can_Manager::toCan(GetCanMsgReq& req, GetCanMsgRes& res){
    ROS_INFO_ONCE("publishing to 'sent_messages' topic from");// )%s");//, req.node_name);
    this->send_.publish(req.msg);
    ROS_INFO_ONCE("Published message.");
    res.ack = "sent.";
    return true;
}

// TODO: check if devices are connected using the JSON file
// TODO: connect to publishers
void Can_Manager::setup(){


}

// Method to parse JSON file into a vector.
void parse_json(std::vector<Device_Property> & json_properties, std::string json_file_location) {
    ptree pt;
    boost::property_tree::read_json(json_file_location, pt);
    
    for (ptree::const_iterator it = pt.begin(); it != pt.end(); ++it) {
        int baud, timeout, retry_count;
        uint8_t id;
        std::string msg, rsp;
        bool ignore, convert;
        // If this is non-auvic made serial device
        size_t size_of_message = 0;
        bool big_endian_message = true;
        size_t size_of_response = 0; 
        bool big_endian_response = true;
        try {
            id = it->second.get<uint8_t>("device_id");
            ignore = it->second.get<bool>("ignore");
            baud = it->second.get<int>("baud");
            msg = it->second.get<std::string>("ack_message");
            rsp = it->second.get<std::string>("ack_response");
            timeout = it->second.get<int>("timeout");
	        retry_count = it->second.get<int>("retry_count");
            convert = it->second.get<bool>("convert_to_bytes");
            if (convert) {
                size_of_message = it->second.get<size_t>("size_of_message");
                size_of_response = it->second.get<size_t>("size_of_response");
                big_endian_message = it->second.get<bool>("big_endian_message");
                big_endian_response = it->second.get<bool>("big_endian_response");
            }
        } catch (...) {
            throw std::runtime_error("Failed to parse " + it->first);
        }
        
        Device_Property new_dev(it->first, id, ignore, msg, rsp, baud, timeout, retry_count,
            convert, size_of_message, size_of_response, big_endian_message, big_endian_response);
        json_properties.push_back(new_dev);
    }
}

int main(int argc, char ** argv) {
    // Setup ROS stuff
    ros::init(argc, argv, "can_manager");
    // "~" is a private namespace. so it cant talk with the socket_bridge topics.
    // "" is used to access the socket_bridge topics 'sent_messages' and 'received_messages'
    ros::NodeHandle nh("");//n("peripherals"), nh("");
    
    // Create ptree structure from JSON file
    std::string json_file_location;
    nh.getParam("devices_json_location", json_file_location);
    std::vector<Device_Property> json_properties;
    parse_json(json_properties, json_file_location);
    
    // can_manager begins
    Can_Manager can_manager(json_properties, &nh);
    can_manager.setup();
    ros::spin();
    return 0;
}
    