#include <ros/ros.h>
#include <iostream>
#include "motor_controller/movement_command.h"
#include "motor_controller/keyboard.h"

using keyboard_msg = motor_controller::keyboard;

class message_reciever {
public:
    void setMessage(const keyboard_msg message) {this->direction = message.direction; this->recieved_new_msg = true; std::cout << "test\n";}
    int getMessage() {this->recieved_new_msg = false; return this->direction;}
    bool recieved() {return this->recieved_new_msg;}
private:
    int direction;
    bool recieved_new_msg = false;
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "motor_controller_client");
    ros::NodeHandle nh("~");
    message_reciever msg;
    ros::Subscriber sub = nh.subscribe<motor_controller::keyboard>("/rov_keyboard/keyboard_input", 20, &message_reciever::setMessage, &msg);

    ros::ServiceClient forward = nh.serviceClient<motor_controller::movement_command>("/auv_nav/MoveForward");
    ros::ServiceClient backward = nh.serviceClient<motor_controller::movement_command>("/auv_nav/MoveBackward");
    ros::ServiceClient left   = nh.serviceClient<motor_controller::movement_command>("/auv_nav/MoveLeft");
    ros::ServiceClient right       = nh.serviceClient<motor_controller::movement_command>("/auv_nav/MoveRight");
    ros::ServiceClient rotateLeft   = nh.serviceClient<motor_controller::movement_command>("/auv_nav/RotateCounterClockwise");
    ros::ServiceClient rotateRight       = nh.serviceClient<motor_controller::movement_command>("/auv_nav/RotateClockwise");
    ros::ServiceClient up          = nh.serviceClient<motor_controller::movement_command>("/auv_nav/MoveUp");
    ros::ServiceClient down          = nh.serviceClient<motor_controller::movement_command>("/auv_nav/MoveDown");
    ros::ServiceClient stop  = nh.serviceClient<motor_controller::movement_command>("/auv_nav/stopMotors");
    
    motor_controller::movement_command srv;

    while (ros::ok())
    {
        if (msg.recieved())
        {
            int kb_msg = msg.getMessage();
            
            switch(kb_msg)
            {
            case motor_controller::keyboard::Forward:
                forward.call(srv);
                break;

            case motor_controller::keyboard::Backward:
                backward.call(srv);
                break;

            case motor_controller::keyboard::Left:
                left.call(srv);
                break;
                   
            case motor_controller::keyboard::Right:
                right.call(srv);
                break;

            case motor_controller::keyboard::RotateCCW:
                rotateLeft.call(srv);
                break;

            case motor_controller::keyboard::RotateCW:
                rotateRight.call(srv);
                break;

            case motor_controller::keyboard::Up:
                up.call(srv);
                break;

            case motor_controller::keyboard::Down:
                down.call(srv);
                break;

            default:
                stop.call(srv);
                break;
                
            }
            
        }
        ros::spinOnce();
    }
    
  return 0;
}
