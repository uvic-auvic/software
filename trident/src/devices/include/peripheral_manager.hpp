#ifndef PERIPHERAL_MANAGER_HPP
#define PERIPHERAL_MANAGER_HPP
//C++ library
#include <memory>
#include <string>
#include <algorithm>
#include <map>
// ROS kernal
#include <ros/ros.h>
#include <ros/console.h>
#include "can_msgs/Frame.h"

//auvic library
#include "auvic_msgs/Monitor_Jetson_To_World.h"

/*
The peripheral manager is a class that stores all the data from the CAN bus. The 
members are static, meaning every instance of Peripherals will have the same data 
available to them. TO access that data, all you need is to call 
'Peripherals::PB_active'. Only do this for the inherited classes. Use 'sailorMan' 
to get data.

To send data to the CAN bus, Use the 'sailor' node. THat node sends messages to the 
bus and also returns information from the Periherals class.

*/
class Peripherals{
public:
    Peripherals(){};
    ~Peripherals(){};

    // Contain the state of the nodes on the bus
    enum devices{ Powerboard, Motorcontroller, Imu, Dvl, Hydrophone, Torpedo, Grabber, Dropper, Lcd_Board, Jetson};
    enum device_status{online, offline, standby}; //if a node is on standby on the network check, disable it.
    typedef std::map<devices, device_status> Network_Manager;
    static Network_Manager device_list;
    static device_status get_Powerboard_status();
    static device_status get_Motorcontroller_status();
    static device_status get_Imu_status();
    static device_status get_Dvl_status();
    static device_status get_Hydrophone_status();
    static device_status get_Torpedo_status();
    static device_status get_Grabber_status();
    static device_status get_Dropper_status();
    static device_status get_Lcd_status();
    static device_status get_Jetson_status();
    // end

    // add variables from CAN bus below to store data
    //...
};

Peripherals::Network_Manager Peripherals::device_list = {
    {Powerboard, offline}, 
    {Motorcontroller, offline},
    {Imu, offline},
    {Dvl, offline},
    {Hydrophone, offline},
    {Torpedo, offline},
    {Grabber, offline},
    {Dropper, offline},
    {Lcd_Board, offline},
    {Jetson, online},
};
Peripherals::device_status Peripherals::get_Jetson_status(){
    return Peripherals::device_list.at(Peripherals::devices::Jetson);
};
Peripherals::device_status Peripherals::get_Lcd_status(){
    return Peripherals::device_list.at(Peripherals::devices::Lcd_Board);
};
Peripherals::device_status Peripherals::get_Dropper_status(){
    return Peripherals::device_list.at(Peripherals::devices::Dropper);
};
Peripherals::device_status Peripherals::get_Grabber_status(){
    return Peripherals::device_list.at(Peripherals::devices::Grabber);
};
Peripherals::device_status Peripherals::get_Torpedo_status(){
    return Peripherals::device_list.at(Peripherals::devices::Torpedo);
};
Peripherals::device_status Peripherals::get_Hydrophone_status(){
    return Peripherals::device_list.at(Peripherals::devices::Hydrophone);
};
Peripherals::device_status Peripherals::get_Dvl_status(){
    return Peripherals::device_list.at(Peripherals::devices::Dvl);
};
Peripherals::device_status Peripherals::get_Imu_status(){
    return Peripherals::device_list.at(Peripherals::devices::Imu);
};
Peripherals::device_status Peripherals::get_Motorcontroller_status(){
    return Peripherals::device_list.at(Peripherals::devices::Motorcontroller);
};
Peripherals::device_status Peripherals::get_Powerboard_status(){
    return Peripherals::device_list.at(Peripherals::devices::Powerboard);
};

/****************************************************************************/
class Powerboard: public Peripherals {
public:
    Powerboard(ros::NodeHandle & n_auvic);
    ~Powerboard();
    void get_PB_active(const can_msgs::Frame::ConstPtr& msg);
    void get_PB_envData(const can_msgs::Frame::ConstPtr& msg);
    void get_PB_battVoltages(const can_msgs::Frame::ConstPtr& msg);
    void get_PB_battCurrents(const can_msgs::Frame::ConstPtr& msg);
    // TODO: set 
private:
    ros::Subscriber PB_deviceName, PB_envData, PB_battVoltages, PB_battCurrents;
    std::string node_name = "powerboard";
        
};

class Motorcontroller: public Peripherals{
public:
    Motorcontroller(ros::NodeHandle & n_auvic);
    ~Motorcontroller();
    void get_MC_active(const can_msgs::Frame::ConstPtr& msg);
    /// TODO: See Polaris repo for implementation
    void get_MC_motorRPMLow(const can_msgs::Frame::ConstPtr& msg);
    /// TODO: See Polaris repo for implementation
    void get_MC_motorRPMHigh(const can_msgs::Frame::ConstPtr& msg);
    /// TODO: See Polaris repo for implementation
    void get_MC_ISOTP(const can_msgs::Frame::ConstPtr& msg);
private:
    ros::Subscriber MC_deviceName, MC_motorRPMLow, MC_motorRPMHigh, MC_ISOTP;
    std::string node_name = "motorcontroller";
};

class Imu: public Peripherals{
public:
    Imu(ros::NodeHandle & n_auvic);
    ~Imu();
    
    void get_IMU_active(const can_msgs::Frame::ConstPtr& msg);
    /// TODO: See Polaris repo for implementation
    void get_temperature(const can_msgs::Frame::ConstPtr& msg);
    /// TODO: See Polaris repo for implementation
    void get_euler_stable(const can_msgs::Frame::ConstPtr& msg);
    /// TODO: See Polaris repo for implementation
    void get_mag_accel_gyro_stable(const can_msgs::Frame::ConstPtr& msg);
    /// TODO: See Polaris repo for implementation
    void get_mag_accel_gyro(const can_msgs::Frame::ConstPtr& msg);
    /// TODO: See Polaris repo for implementation
    void get_velocity(const can_msgs::Frame::ConstPtr& msg);
    /// TODO: See Polaris repo for implementation
    void set_velocity(const can_msgs::Frame::ConstPtr& msg);
    /// TODO: See Polaris repo for implementation
    void update_velocity(const can_msgs::Frame::ConstPtr& msg);
private:
    /// TODO: do a CHECKSUM test
    void verify_response();
    std::string node_name = "imu";
};

class Grabber: public Peripherals{
public:
    Grabber(ros::NodeHandle & n_auvic);
    ~Grabber();
    void get_GRA_active(const can_msgs::Frame::ConstPtr& msg);
private:
    std::string node_name = "grabber";
};

class Dropper: public Peripherals{
public:
    Dropper(ros::NodeHandle & n_auvic);
    ~Dropper();
    void get_DRO_active(const can_msgs::Frame::ConstPtr& msg);
private:
    std::string node_name = "dropper";
};

class Dvl: public Peripherals{
public:
    Dvl(ros::NodeHandle & n_auvic);
    ~Dvl();
    void get_DVL_active(const can_msgs::Frame::ConstPtr& msg);
private:
    std::string node_name = "dvl";
};

class Torpedo: public Peripherals{
public:
    Torpedo(ros::NodeHandle & n_auvic);
    ~Torpedo();
    void get_TOR_active(const can_msgs::Frame::ConstPtr& msg);
private:
    std::string node_name = "torpedo";
};

class Hydrophone: public Peripherals{
public:
    Hydrophone(ros::NodeHandle & n_auvic);
    ~Hydrophone();
    void get_HYD_active(const can_msgs::Frame::ConstPtr& msg);
    /// TODO: See Polaris repo for implementation
    void get_raw_data(const can_msgs::Frame::ConstPtr& msg);
    /// TODO: See Polaris repo for implementation
    void get_fft_data(const can_msgs::Frame::ConstPtr& msg);
    /// TODO: See Polaris repo for implementation
    void get_hydro_phases(const can_msgs::Frame::ConstPtr& msg);


private:
    /// TODO:
    void acquire_hydro_data();
    /// TODO:
    void compute_fft();
    std::string node_name = "hydrophone";
};

class Lcd_Board: public Peripherals{
public:
    Lcd_Board(ros::NodeHandle & n_auvic);
    ~Lcd_Board();
    void get_LCD_active(const can_msgs::Frame::ConstPtr& msg);
private:
    std::string node_name = "lcd_board";
};



#endif /* PERIPHERAL_MANAGER_HPP */