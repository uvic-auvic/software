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
    monitor::Can_Manager can_manager(json_properties, &nh);
    
    ros::spin();
    return 0;
}
    