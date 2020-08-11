// auvic libraries
#include <peripheral_manager.hpp>

// @param1 nodehandle to speak with auvic topics
Hydrophone::Hydrophone(ros::NodeHandle & n_auvic){}

Hydrophone::~Hydrophone(){};

void Hydrophone::get_HYD_active(const can_msgs::Frame::ConstPtr& msg){
    ROS_INFO("Hydrophone is active!");
    Peripherals::device_list.at(devices::Hydrophone) = Peripherals::device_status::online;
    if( Peripherals::device_list.at(devices::Hydrophone) == Peripherals::device_status::online){
        ROS_INFO_ONCE("DEVICES/Hydrophone_node: Hydrophone is active!");
    }
    else {
        ROS_WARN_ONCE("DEVICES/Hydrophone_node: Hydrophone is not active");
    }
}
/// TODO: Send Message ID and data to send_frame
void get_raw_data(const can_msgs::Frame::ConstPtr& msg){};
/// TODO: Send Message ID and data to send_frame
void get_fft_data(const can_msgs::Frame::ConstPtr& msg){};
/// TODO: Send Message ID and data to send_frame
void get_hydro_phases(const can_msgs::Frame::ConstPtr& msg){};


/// TODO:
void acquire_hydro_data(){};
/// TODO:
void compute_fft(){};