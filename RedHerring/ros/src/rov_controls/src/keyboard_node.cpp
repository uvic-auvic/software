#include <ros/ros.h>
#include "std_msgs/String.h"
#include <signal.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <ros/console.h>
#include <string.h>
#include "motor_controller/keyboard.h"

//Globals
int kfd = 0;
struct termios cooked, raw;


void quit(int sig)
{
    (void)sig;
    tcsetattr(kfd, TCSANOW, &cooked);
    ROS_INFO("Shutting down");
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    char KB_char = '\0';
    //Get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    // ROS Setup
    ros::init(argc,argv,"rov_keyboard");
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<motor_controller::keyboard>("keyboard_input", 20);
    motor_controller::keyboard keyboard_input;
    signal(SIGINT, quit);
    ROS_INFO("KEYBOARD INPUT NODE ONLINE");    

    while(ros::ok())
    {
        // get the next event from the keyboard
        if(read(kfd, &KB_char, 1) < 0)
        {
          ROS_ERROR("read():");
          //exit(-1);
        }

        ROS_INFO("Code: %c %x", KB_char, KB_char);
        keyboard_input.direction = KB_char;
        pub.publish(keyboard_input);
        
        ros::spinOnce();
    }

    return 0;
}
