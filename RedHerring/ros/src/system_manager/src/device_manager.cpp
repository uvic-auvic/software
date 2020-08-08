#include <ros/ros.h>
#include <serial/serial.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <dirent.h>
#include "system_manager/device_manager.h"

// Map for storing file descripters
std::map<std::string, std::string> file_descriptors;
serial::Serial * serial_conn = nullptr;

void closeSerialConnection()
{
    if (serial_conn != nullptr)
    {
        serial_conn->close();
        delete serial_conn;
        serial_conn = nullptr;
    }
}

void setupSerialConnection(std::string fd, int baud_rate = 9600, int timeout = 1000)
{
    closeSerialConnection();
    //ROS_INFO("Creating serial connection on port: %s" % fd.c_str());
    serial_conn = new serial::Serial(fd, (u_int32_t) baud_rate, serial::Timeout::simpleTimeout(timeout));
}

std::string serial_out(std::string serial_out_string, bool expect_response)
{
    serial_conn->write(serial_out_string);

    if (!expect_response)
    {
        return "";
    }

    return serial_conn->readline(65536ul, "\r\n");
}

void getNameOfDevice(std::string fd)
{
    setupSerialConnection(fd);
    std::string rid_result = serial_out("RID\n", true);
    if (!rid_result.empty()) {
        file_descriptors[rid_result] = fd;
        ROS_INFO("Device on %s identifies as %s", fd.c_str(), rid_result.c_str() );
    }
    
}

bool getFileDescriptor(system_manager::device_manager::Request &req, system_manager::device_manager::Response &res)
{
    res.file_desc = file_descriptors[req.device];
    return true;
}

int main(int argc, char ** argv) {
    // Setup ROS stuff
    ros::init(argc, argv, "device_manager");
    ros::NodeHandle nh("~");

    // List all the files in the /dev directory and store in a list
    std::string dirname("/dev");
    DIR * pd = opendir(dirname.c_str());

    // get all the ttyUSB's'
    struct dirent * pdirent;    
    while ((pdirent = readdir(pd)) != NULL) {
        std::string fd(pdirent->d_name);
        if (fd.find("ttyUSB") == 0) {
            getNameOfDevice("/dev/" + fd);
        }
    }
    closedir(pd);
    closeSerialConnection();

    ros::ServiceServer getFdFromName = nh.advertiseService("getDeviceFd", getFileDescriptor);
    ros::spin();
    
    return 0;
}
