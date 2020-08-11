/********************************************************************
 * @file /src/joystick_node.cpp
 * @brief The program that interfaces with the joystick controller
 * @date February 2017
/********************************************************************/

/********************************************************************
 * Includes
/********************************************************************/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <unistd.h>
#include <iostream>

#include "joystick.h"

/********************************************************************
 * Implementation [Main]
 * @Args     argc is the number of command-line arguments provided
 * @Args     argv is a pointer to the argument strings
/********************************************************************/
int main(int argc, char **argv){
    ros::init(argc, argv, "joystick_node");
    ros::NodeHandle nh("~");

    /* Launch Parameters*/
    std::string joy_fd, joy_pub_name, msg;
    int looping_rate, sleep_time, timeout_time;

    /* get Parameters */
     nh.getParam("joy_fd", joy_fd);
     nh.getParam("joy_pub_name", joy_pub_name);
     nh.getParam("looping_rate", looping_rate);
     nh.getParam("sleep_time", sleep_time);
     nh.getParam("timeout_time", timeout_time);

    /* Setup Node */
    ros::Publisher joy_pub = nh.advertise<std_msgs::String>(joy_pub_name, 10);
    Joystick joystick(joy_fd);
    ros::Rate loop_rate(looping_rate);

    /* Sleep for 1s if no Joystick is found. Shutdown node after 60s */
    int timeout_counter = timeout_time;
    while(!joystick.isFound())
    {
        msg = "Joystick not found on " + joy_fd + "s. Waiting for " + std::to_string(timeout_counter--);
        ROS_INFO("%s", msg.c_str());
        usleep(sleep_time);

        if (!timeout_counter) {
            msg = "No Joystick found after " + std::to_string(timeout_time) + "s. Closing Joystick Node";
            ROS_INFO("%s", msg.c_str());
            return -1;
        }

    }

    /* Inform everyone that we just connected the Joystick */
    msg = "Joystick found";
    ROS_INFO("%s", msg.c_str());
    ros::spinOnce();

    /* Poll for Joystick Samples */
    while(ros::ok()) {
        JoystickEvent event;

        if (joystick.sample(&event)) {
            std_msgs::String msgs;
            std::stringstream ss;

            int eventNumber = (int) event.number;
            int eventValue  = (int) event.value;

            if (event.isButton()){
                ss << "Button;" << (eventNumber + 1) << ";" << eventValue;
            } else if (event.isAxis()) {
                ss << "Axis;" << eventNumber << ";" << eventValue;
            }

            msgs.data = ss.str();
            ROS_INFO("%s", msgs.data.c_str());
            joy_pub.publish(msgs);

        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("END");
    return 0;
}
