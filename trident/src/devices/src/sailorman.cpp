#include <sailorman.hpp>


SailorMan::SailorMan(ros::NodeHandle & n_auvic, ros::NodeHandle & server){
    // Topics to publish messages to
    TRIDENT_deviceName = n_auvic.advertise<auvic_msgs::Monitor_Jetson_To_World>("protocol_MID_TRIDENT_deviceName", 10);
    TRIDENT_motorSetSpeed = n_auvic.advertise<auvic_msgs::Monitor_Jetson_To_World>("protocol_MID_TRIDENT_motorSetSpeed", 10);
    TRIDENT_powerEnable = n_auvic.advertise<auvic_msgs::Monitor_Jetson_To_World>("protocol_MID_TRIDENT_powerEnable", 10);
    TRIDENT_MCISOTP = n_auvic.advertise<auvic_msgs::Monitor_Jetson_To_World>("protocol_MID_TRIDENT_MCISOTP", 10);

    // App layer interface
    To_World_Service = server.advertiseService("Send_Data_To_World", &SailorMan::Send_Data_To_World, this);
    Get_Data_Service = server.advertiseService("Get_Data_From_Peripherals", &SailorMan::Get_Data_From_Peripherals, this);
};

SailorMan::SailorMan(ros::NodeHandle & n_auvic){
    // Topics to publish messages to
    TRIDENT_deviceName = n_auvic.advertise<auvic_msgs::Monitor_Jetson_To_World>("protocol_MID_TRIDENT_deviceName", 10);
    TRIDENT_motorSetSpeed = n_auvic.advertise<auvic_msgs::Monitor_Jetson_To_World>("protocol_MID_TRIDENT_motorSetSpeed", 10);
    TRIDENT_powerEnable = n_auvic.advertise<auvic_msgs::Monitor_Jetson_To_World>("protocol_MID_TRIDENT_powerEnable", 10);
    TRIDENT_MCISOTP = n_auvic.advertise<auvic_msgs::Monitor_Jetson_To_World>("protocol_MID_TRIDENT_MCISOTP", 10);

    // periodic publishing: add jetson messages below
    list_of_publishing_topics.push_back("protocol_MID_TRIDENT_deviceName");
    //list_of_publishing_topics.push_back("protocol_MID_TRIDENT_motorSetSpeed");
    //list_of_publishing_topics.push_back("protocol_MID_TRIDENT_powerEnable");
    //list_of_publishing_topics.push_back("protocol_MID_TRIDENT_MCISOTP");

    // periodic timer
    NM_time = n_auvic.createTimer(ros::Duration(10), &SailorMan::Run_Network_Management, this);
};

SailorMan::~SailorMan(){};

//Logic: get data to send. Find what topic to publish, save a copy in peripherals, and then publish.
bool SailorMan::Send_Data_To_World(Send_Data_Request &req, Send_Data_Response &res) {
    // DESIGN CHOICE: should we drop messages for devices that are offline?
    // Be careful about mixing the service up with the message. Topics take messages, not services.
    // Get the topic where the message is going
    std::string topic_name = req.input.message_name.data;
    if( topic_name == "protocol_MID_TRIDENT_deviceName"){
        ROS_INFO_ONCE("DEVICES/SAILORMAN_NODE: publishing to [%s]!", topic_name.c_str());
        Peripherals::device_list.at(Peripherals::devices::Jetson) = Peripherals::device_status::online;
        TRIDENT_deviceName.publish(req.input);
    } else if( topic_name == "protocol_MID_TRIDENT_motorSetSpeed"){
        ROS_INFO_ONCE("DEVICES/SAILORMAN_NODE: publishing to [%s]!", topic_name.c_str());
        // backup data
        TRIDENT_motorSetSpeed.publish(req.input);
    } else if( topic_name == "protocol_MID_TRIDENT_powerEnable"){
        ROS_INFO_ONCE("DEVICES/SAILORMAN_NODE: publishing to [%s]!", topic_name.c_str());
        // backup data
        TRIDENT_powerEnable.publish(req.input);
    } else if( topic_name == "protocol_MID_TRIDENT_MCISOTP"){
        ROS_INFO_ONCE("DEVICES/SAILORMAN_NODE: publishing to [%s]!", topic_name.c_str());
        // backup data
        TRIDENT_MCISOTP.publish(req.input);
    } else {
        ROS_ERROR("DEVICES/SAILORMAN_NODE: TOPIC NAME WAS INCORRECT. MESSAGE IS DROPPED");
        res.output.data = "dropped.";
        return false;
    }
    // Acknolwedge
    res.output.data = "sent.";
    return true;
};

// Logic: Get topic name, collect data from peripherals, return via res.
bool SailorMan::Get_Data_From_Peripherals(Get_Data_Request &req, Get_Data_Response &res){
    std::string topic_name = req.message_name.data;
    if( topic_name == "protocol_MID_MC_deviceName"){
        ROS_INFO_ONCE("DEVICES/SAILORMAN_NODE: getting [%s]!", topic_name.c_str());
        bool output = Peripherals::device_list.at(Peripherals::devices::Motorcontroller);
        int i = 0;
        while(i < 8){ res.data[i++] = output? 1 : 0;};
    } else if( topic_name == "protocol_MID_MC_motorRPMLow"){
        ROS_INFO_ONCE("DEVICES/SAILORMAN_NODE: getting [%s]!", topic_name.c_str());
    } else if( topic_name == "protocol_MID_MC_motorRPMHigh"){
        ROS_INFO_ONCE("DEVICES/SAILORMAN_NODE: getting [%s]!", topic_name.c_str());
    } else if( topic_name == "protocol_MID_MC_ISOTP"){
        ROS_INFO_ONCE("DEVICES/SAILORMAN_NODE: getting [%s]!", topic_name.c_str());
    } else if( topic_name == "protocol_MID_PB_deviceName"){
        ROS_INFO_ONCE("DEVICES/SAILORMAN_NODE: getting [%s]!", topic_name.c_str());
        bool output = Peripherals::device_list.at(Peripherals::devices::Powerboard);
        int i = 0;
        while(i < 8){ res.data[i++] = output?1:0;};     
    } else if( topic_name == "protocol_MID_PB_envData"){
        ROS_INFO_ONCE("DEVICES/SAILORMAN_NODE: getting [%s]!", topic_name.c_str());
    } else if( topic_name == "protocol_MID_PB_battVoltages"){
        ROS_INFO_ONCE("DEVICES/SAILORMAN_NODE: getting [%s]!", topic_name.c_str()); 
    } else if( topic_name == "protocol_MID_PB_battCurrents"){
        ROS_INFO_ONCE("DEVICES/SAILORMAN_NODE: getting [%s]!", topic_name.c_str()); 
    } else {
        ROS_ERROR("DEVICES/SAILORMAN_NODE: TOPIC NAME WAS INCORRECT. REQUEST DROPPED!");
        return false;
    }
    return true;
};

// Runs every 10 seconds to reset the device active list.
// If a node is properly functioning, it should become active
// sometime later, otherwise, give the device an offline status.
void SailorMan::Run_Network_Management(const ros::TimerEvent&){
    // online->standby->offline
    for(auto it = Peripherals::device_list.begin(); it != Peripherals::device_list.end(); it++){
        if( it->second == Peripherals::device_status::standby) {
            it->second = Peripherals::device_status::offline;
        }
        else if( it->second == Peripherals::device_status::online) {
            it->second = Peripherals::device_status::standby;
        }    
        // keep jetson online
        if(it->first == Peripherals::devices::Jetson){
            it->second = Peripherals::device_status::online;
        }
    }
}



