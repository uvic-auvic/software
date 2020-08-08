/********************************************************************
 * @file /src/motor_controller.cpp
 * @brief The program that interfaces with the motor controllers
 * @date February 2017
/********************************************************************/

/********************************************************************
 * Includes
/********************************************************************/
#include <ros/ros.h>
#include <string>
#include <serial/serial.h>
#include <math.h>
#include <thread>
#include <chrono>
#include "motor_controller/motor_command.h"
#include "motor_controller/movement_command.h"
#include "motor_controller/position.h"

enum movement_mode 
{
    FAR_AWAY,               //0
    STRAIGHT_AHEAD,         //1
    CLOSE_VICINITY          //2
};

enum motor_numbers
{
    motor_front,            //0
    motor_side,             //1
    motor_back              //2
};

class motor_client
{
public:
    ros::NodeHandle nh;
    ros::ServiceClient forward;
    ros::ServiceClient reverse;
    ros::ServiceClient stopMotors;
    ros::ServiceClient stopMotor;
    ros::ServiceClient getRPM;

    motor_client(std::string motor_conn_name) :
        nh(ros::NodeHandle("~")),
        forward(nh.serviceClient<motor_controller::motor_command>("/" + motor_conn_name + "/setMotorForward")),
        reverse(nh.serviceClient<motor_controller::motor_command>("/" + motor_conn_name + "/setMotorReverse")),
        stopMotors(nh.serviceClient<motor_controller::motor_command>("/" + motor_conn_name + "/stopAllMotors")),
        stopMotor(nh.serviceClient<motor_controller::motor_command>("/" + motor_conn_name + "/stopMotor")),
        getRPM(nh.serviceClient<motor_controller::motor_command>("/" + motor_conn_name + "/getRPM"))
    {
        reverse_motor_multiplier = nh.getParam("reverse_multiplier", reverse_motor_multiplier);
    }

    motor_controller::motor_command generateMotorCommand(int motor_number, int command_param)
    {
        motor_controller::motor_command srv;
        srv.request.motor_number = motor_number;
        srv.request.command_param = command_param;
        return srv;
    }

    void setBothForward(int power, bool reduce_power=false)
    {
        if (reduce_power) {
            power = getReducedPower(power);
        }
        motor_controller::motor_command mf = generateMotorCommand(motor_front, power);
        motor_controller::motor_command mb = generateMotorCommand(motor_back, power);
        this->forward.call(mf);
        this->forward.call(mb);
    }

    void setBothBackward(int power)
    {
        motor_controller::motor_command mf = generateMotorCommand(motor_front, power);
        motor_controller::motor_command mb = generateMotorCommand(motor_back, power);
        this->reverse.call(mf);
        this->reverse.call(mb);
    }

    void setFrontForwardBackBackward(int base_power)
    {
        int reduced_power = getReducedPower(base_power);
        motor_controller::motor_command mf = generateMotorCommand(motor_front, reduced_power);
        motor_controller::motor_command mb = generateMotorCommand(motor_back, base_power);
        this->forward.call(mf);
        this->reverse.call(mb);
    }

    void setFrontBackwardBackForward(int base_power)
    {
        motor_controller::motor_command mf = generateMotorCommand(motor_front, base_power);
        int reduced_power = getReducedPower(base_power);
        motor_controller::motor_command mb = generateMotorCommand(motor_back, reduced_power);
        this->reverse.call(mf);
        this->forward.call(mb);
    }

    void up(int power)
    {
        setMotorForward(motor_side, power);
    }

    void down(int power)
    {
        setMotorReverse(motor_side, power);
    }

    void setMotorForward(int motor, int power)
    {
        motor_controller::motor_command mup = generateMotorCommand(motor, power);
        this->forward.call(mup);
    }

    void setMotorReverse(int motor, int power)
    {
        motor_controller::motor_command mdown = generateMotorCommand(motor, power);
        this->reverse.call(mdown);
    }

    void setMotorsStop()
    {
        motor_controller::motor_command stop_msg = generateMotorCommand(0, 0);
        this->stopMotors.call(stop_msg);
    }

    int getReducedPower(int speed)
    {
        return (int) speed * reverse_motor_multiplier;
    }

private:
    float reverse_motor_multiplier;
};

class movement_controller
{
public:
    movement_controller() :
        nh(ros::NodeHandle("~")),
        left_controller(motor_client("motor_left")),
        right_controller(motor_client("motor_right"))
    {
        nh.getParam("default_power", default_power);
        nh.getParam("max_speed_for_default", max_ms_at_default);
    }

    // Service Call Definitions
    bool Forward( motor_controller::movement_command::Request &req,
                  motor_controller::movement_command::Response &res)
    {
        left_controller.setBothForward(default_power);
        right_controller.setBothForward(default_power);
        return true;
    }

    bool Backward( motor_controller::movement_command::Request &req,
                   motor_controller::movement_command::Response &res)
    {
        left_controller.setBothBackward(default_power);
        right_controller.setBothBackward(default_power);
        return true;
    }

    bool Left( motor_controller::movement_command::Request &req,
               motor_controller::movement_command::Response &res)
    {
        left_controller.setFrontBackwardBackForward(default_power);
        right_controller.setFrontForwardBackBackward(default_power);
        return true;
    }

    bool Right( motor_controller::movement_command::Request &req,
                motor_controller::movement_command::Response &res)
    {
        right_controller.setFrontBackwardBackForward(default_power);
        left_controller.setFrontForwardBackBackward(default_power);
        return true;
    }

    bool RotateClockwise( motor_controller::movement_command::Request &req,
                          motor_controller::movement_command::Response &res)
    {
        left_controller.setBothForward(default_power, true);
        right_controller.setBothBackward(default_power);
        return true;
    }

    bool RotateCounterClockwise( motor_controller::movement_command::Request &req,
                                 motor_controller::movement_command::Response &res)
    {
        right_controller.setBothForward(default_power, true);
        left_controller.setBothBackward(default_power);
        return true;
    }

    bool moveUp( motor_controller::movement_command::Request &req,
                 motor_controller::movement_command::Response &res)
    {
        moveVertical(5, default_power);
        return true;
    }

    bool moveDown( motor_controller::movement_command::Request &req,
                   motor_controller::movement_command::Response &res)
    {
        moveVertical(-5, default_power);
        return true;
    }

    bool MoveToPosition( motor_controller::movement_command::Request &req,
                         motor_controller::movement_command::Response &res)
    {
        // ignore height in these calculations 
        float distance = sqrt(pow(req.pos.x, 2) + pow(req.pos.z, 2));
        movement_mode mode = CLOSE_VICINITY;
        if (distance > 5) // if we're further than 5m
        {                                     
            if (abs(req.pos.x) > 1) // if we need to move more than 1m in x
            {
                mode = FAR_AWAY;
            } 
            else 
            {
                mode = STRAIGHT_AHEAD;
            }
        }

        // boilerplate msg. Not really imporant what the values are (they're ignored)
        motor_controller::movement_command::Request new_req;
        motor_controller::movement_command::Response new_res;

        // Find how long it will take us to ascend
        int time_y = abs(req.pos.y) / max_ms_at_default;
        int time_x = abs(req.pos.x) / max_ms_at_default;
        int time_z = abs(req.pos.z) / max_ms_at_default;


        switch (mode)
        {
            case CLOSE_VICINITY:
            {
                // Moving manually in each direction will be more precise here
                // Left/Right, Up/Down then forwards 
                for (int i=0; i<time_x; i++) // X
                {
                    if (req.pos.x > 0)
                    {
                        this->Right(new_req, new_res);
                    } else
                    {
                        this->Left(new_req, new_res);
                    }
                    this->sleep(900);
                }
                this->stop();

                for (int i=0; i<time_y; i++) // Y
                {
                    this->moveVertical(req.pos.y, default_power);
                    this->sleep(1000);
                }
                this->stop();

                for (int i=0; i<time_z; i++) // z
                {
                    this->Forward(new_req, new_res);
                    sleep(1000);
                }
                this->stop();
            }
            break;

            case FAR_AWAY:
            {
                float x_hat = req.pos.x / distance;
                float y_hat = req.pos.y / distance;

                float x_unit_vector = 2 / distance;
                float y_unit_vector = y_hat - abs(x_hat - x_unit_vector);

                // cramers rule
                int determinant = -2; // assuming all 4 motors are at 90 degree angles
                float motor_top_per = -(x_unit_vector + y_unit_vector) / determinant;
                float motor_bot_per = (y_unit_vector - x_unit_vector) / determinant;
                float lone_motor_per = y_hat - motor_bot_per;
                // determine motor percentages so that they make the correct vector
                int motor_top_power = (int) abs(lone_motor_per/motor_top_per) * default_power;
                int motor_bot_power = (int) abs(motor_bot_per/motor_top_per) * motor_top_power;
                // My piss-poor attempt at arriving at the y position at the same time as the z
                int total_time = distance / max_ms_at_default;
                float time_y_normalized = (float) time_y / total_time;
                int vert_power = (int) time_y_normalized * default_power;
                if (req.pos.x > 0) 
                {   // going right
                    for (int i=0; i<total_time; i++)
                    {
                        left_controller.setMotorForward(motor_front, motor_top_power);
                        left_controller.setMotorReverse(motor_back, motor_bot_power);
                        right_controller.setMotorForward(motor_back, default_power);
                        this->moveVertical(req.pos.z, vert_power);
                        this->sleep(900);
                    }
                } else 
                {   // going left
                    for (int i=0; i<total_time; i++)
                    {
                        right_controller.setMotorForward(motor_front, motor_top_power);
                        right_controller.setMotorReverse(motor_back, motor_bot_power);
                        left_controller.setMotorForward(motor_back, default_power);
                        this->moveVertical(req.pos.z, vert_power);
                        this->sleep(900);
                    }
                }
                stop();
            } 
            break;

            case STRAIGHT_AHEAD:
            {
                // a slightly less piss-poor attempt at slowing down z in order to get to the right position at once
                float time_y_normalizedd = (float) time_y / time_z; 
                int vertical_power = (int) time_y_normalizedd * default_power; 
                
                for (int i=0; i<time_z; i++)
                {
                    this->moveVertical(req.pos.y, vertical_power);
                    this->Forward(new_req, new_res); // just to make sure we're still moving
                    this->sleep(950); // accounting for all the ROS+Motor overhead
                }
                stop();
            }
            break;
        }
        return true;
    }

    void moveVertical(int z_position, int vertical_motor_power)
    {
        if (z_position > 0)
            {
                left_controller.up(vertical_motor_power);
                right_controller.up(vertical_motor_power);
            }
            else
            {
                left_controller.down(vertical_motor_power);
                right_controller.down(vertical_motor_power);
            }
    }

    void stop()
    {
        left_controller.setMotorsStop();
        right_controller.setMotorsStop();
    }

    void sleep(int milliseconds)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
    }

private:
    ros::NodeHandle nh;
    motor_client left_controller;
    motor_client right_controller;
    int default_power;
    float max_ms_at_default;
};


/********************************************************************
 * Implementation [Main]
 * @Args     argc is the number of command-line arguments provided
 * @Args     argv is a pointer to the argument strings
/********************************************************************/
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "auv_nav");
    ros::NodeHandle nh("~");
    movement_controller controller;

    // Services
    ros::ServiceServer MoveForward  = nh.advertiseService("MoveForward", &movement_controller::Forward, &controller);
    ros::ServiceServer MoveBackward = nh.advertiseService("MoveBackward", &movement_controller::Backward, &controller);
    ros::ServiceServer MoveLeft     = nh.advertiseService("MoveLeft", &movement_controller::Left, &controller);
    ros::ServiceServer MoveRight    = nh.advertiseService("MoveRight", &movement_controller::Right, &controller);
    ros::ServiceServer CW           = nh.advertiseService("RotateClockwise", &movement_controller::RotateClockwise, &controller);
    ros::ServiceServer CCW          = nh.advertiseService("RotateCounterClockwise", &movement_controller::RotateCounterClockwise, &controller);
    ros::ServiceServer Move         = nh.advertiseService("MoveToPosition", &movement_controller::MoveToPosition, &controller);
    ros::ServiceServer up           = nh.advertiseService("MoveUp", &movement_controller::MoveToPosition, &controller);
    ros::ServiceServer down         = nh.advertiseService("MoveDown", &movement_controller::MoveToPosition, &controller);
    ros::ServiceServer stop         = nh.advertiseService("stopMotors", &movement_controller::MoveToPosition, &controller);

    /* Wait for callbacks */
    ros::spin();
    return 0;
}
