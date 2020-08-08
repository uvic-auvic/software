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
#include "motor_controller/motor_command.h"
//#include "system_manager/device_manager.h"

/********************************************************************
 * Defines
/********************************************************************/
#define MAX_NUMBER_OF_MOTORS (3)
#define MAX_INDEX_OF_MOTORS (MAX_NUMBER_OF_MOTORS-1)

/********************************************************************
 * Global Variables
/********************************************************************/
serial::Serial * serial_conn = nullptr; //shared between service calls so needs to global

/********************************************************************
 * Implementation [Helper Functions]
/********************************************************************/

/********************************************************************
 * @Name     setupSerialConnection
 * @Args     fd File Descriptor or port of serial device
 * @Args     baud_rate baud_rate of serial device. defaults to 9600
 * @Args     timeout Time before we auto-disconnect during comm.
 * @function Maps the serial_conn object to a specific port so all
 *           we have to do serial_conn->write or read to communicate
/********************************************************************/
void setupSerialConnection(std::string fd, int baud_rate = 9600, int timeout = 1000)
{
    if (serial_conn != nullptr)
    {
        serial_conn->close();
        delete serial_conn;
    }

    ROS_INFO("Creating serial connection on port: %s, Baud Rate: %d with timeout time: %d", fd.c_str(), baud_rate, timeout);
    serial_conn = new serial::Serial(fd, (u_int32_t) baud_rate, serial::Timeout::simpleTimeout(timeout));
}

/********************************************************************
 * @Name     serial_out
 * @Args     serial_out_string is the string to transfer
 * @Args     expect_response tells the function whether to read for a response
 * @function writes to the serial device and returns the response
/********************************************************************/
std::string serial_out(std::string serial_out_string, bool expect_response = true)
{
    serial_conn->write(serial_out_string); // dont really care about size of the message, let the firmware deal with that

    if (!expect_response)
    {
        return "";
    }

    return serial_conn->readline(65536ul,"\r\n");
}

/********************************************************************
 * @Name     parseSerialResponse
 * @Args     response is the string returned from the serial device
 * @Args     expect_response tells the function whether a response exists
 * @function parses the response from the device and determines if
 *           the command was successful
/********************************************************************/
inline bool parseSerialResponse(const std::string & response, bool expect_response)
{
    return (bool) !(expect_response && response == "ERR");
}

/********************************************************************
 * Implementation [Service Calls]
 * @Args     req is the request message sent by the service calller
 * @Args     res is the reponse that we generate depending on the outcome
/********************************************************************/

/********************************************************************
 * @Name     setMotorForward [Service Call]
 * @function Sets motor forward speed
/********************************************************************/
bool setMotorForward(motor_controller::motor_command::Request &req,
                     motor_controller::motor_command::Response &res)
{
    /* Check whether the motor exists */
    if (req.motor_number > MAX_INDEX_OF_MOTORS)
    {
        ROS_ERROR("No Motor %d. Max Motor index is %d", req.motor_number, MAX_INDEX_OF_MOTORS);
        return false;
    }

    /* Create UART string and send to serial device */
    /* String: M${X}F${Arg}\n */
    std::string serial_out_str = "M" + std::to_string(req.motor_number) + "F" + std::string(1, (unsigned char) req.command_param) + "\n";

    /* Sending serial output */
    ROS_INFO("Sending serial output: %s", serial_out_str.c_str());
    res.motor_response = serial_out(serial_out_str, false);
    res.motor_success = parseSerialResponse(res.motor_response, false);
    // The point of the motor_success is to give the control_system a chance to re-adjust as necessary
    // Chances are good though that it will never be used
    // We can come back to this and see how the control system actually ends up being implemented

  return true;
}

/********************************************************************
 * @Name     setMotorReverse
 * @function Sets motor reverse speed
/********************************************************************/
bool setMotorReverse(motor_controller::motor_command::Request &req,
                     motor_controller::motor_command::Response &res)
{
    /* Check whether the motor exists */
    if (req.motor_number > MAX_INDEX_OF_MOTORS)
    {
        ROS_ERROR("No Motor %d. Max Motor index is %d", req.motor_number, MAX_INDEX_OF_MOTORS);
        return false;
    }

    /* Create UART string and send to serial device */
    /* String: M${X}R${Arg}\n */
    std::string serial_out_str = "M" + std::to_string(req.motor_number) + "R" + std::string(1, (unsigned char) req.command_param) + "\n";

    /* Sending serial output */
    ROS_INFO("Sending serial Output: %s", serial_out_str.c_str());
    res.motor_response = serial_out(serial_out_str, false);
    res.motor_success = parseSerialResponse(res.motor_response, false);

  return true;
}

/********************************************************************
 * @Name     stopAllMotors
 * @function stops all the motors
/********************************************************************/
bool stopAllMotors(  motor_controller::motor_command::Request &req,
                     motor_controller::motor_command::Response &res)
{
    ROS_INFO("Stopping all motors");
    /* String: STP\n */
    std::string serial_out_str = "STP\n" ;

    /* Sending serial output */
    ROS_INFO("Serial Output: %s", serial_out_str.c_str());
    res.motor_response = serial_out(serial_out_str, true);
    res.motor_success = parseSerialResponse(res.motor_response, true);

    /* Default to true */
    res.motor_success = true;

  return true;
}

/********************************************************************
 * @Name     stopMotor
 * @function Sets only 1 motor
/********************************************************************/
bool stopMotor(      motor_controller::motor_command::Request &req,
                     motor_controller::motor_command::Response &res)
{
    /* Check whether the motor exists */
    if (req.motor_number > MAX_INDEX_OF_MOTORS)
    {
        ROS_ERROR("No Motor %d. Max Motor index is %d", req.motor_number, MAX_INDEX_OF_MOTORS);
        return false;
    }

    /* Create UART string and send to serial device */
    /* String: SM${X}\n */
    std::string serial_out_str = "SM" + std::to_string(req.motor_number) + "\n";

    /* Sending serial output */
    ROS_INFO("Serial Output: %s", serial_out_str.c_str());
    res.motor_response = serial_out(serial_out_str, true);
    res.motor_success = parseSerialResponse(res.motor_response, true);

  return true;
}

/********************************************************************
 * @Name     getRPM
 * @function Returns the RPM of one motor
/********************************************************************/
bool getRPM(         motor_controller::motor_command::Request &req,
                     motor_controller::motor_command::Response &res)
{
    /* Check whether the motor exists */
    if (req.motor_number > MAX_INDEX_OF_MOTORS)
    {
        ROS_ERROR("No Motor %d. Max Motor index is %d", req.motor_number, MAX_INDEX_OF_MOTORS);
        return false;
    }

    /* Create UART string and send to serial device */
    /* String: RV${X}\n */
    std::string serial_out_str = "RV" + std::to_string(req.motor_number) + "\n";

    /* Sending serial output */
    ROS_INFO("Serial Output: %s", serial_out_str.c_str());
    res.motor_response = serial_out(serial_out_str, true);
    res.motor_success = parseSerialResponse(res.motor_response, true);

  return true;
}

/********************************************************************
 * @Name     setPWM
 * @function Manuallt sets the PWM duty cycle of a motor
/********************************************************************/
bool setPWM(         motor_controller::motor_command::Request &req,
                     motor_controller::motor_command::Response &res)
{
    /* Check whether the motor exists */
    if (req.motor_number > MAX_INDEX_OF_MOTORS)
    {
        ROS_ERROR("No Motor %d. Max Motor index is %d", req.motor_number, MAX_INDEX_OF_MOTORS);
        return false;
    }

    /* Create UART string and send to serial device */
    /* String: PW${X}{Arg}\n */
    std::string serial_out_str = "PW" + std::to_string(req.motor_number) + std::string(1, (unsigned char) req.command_param) + "\n";

    /* Sending serial output */
    ROS_INFO("Serial Output: %s", serial_out_str.c_str());
    res.motor_response = serial_out(serial_out_str, false);
    res.motor_success = parseSerialResponse(res.motor_response, false);

  return true;
}

/********************************************************************
 * @Name     calibrateMotor
 * @function Manually calibrates a motor
/********************************************************************/
bool calibrateMotor( motor_controller::motor_command::Request &req,
                     motor_controller::motor_command::Response &res)
{
    /* Check whether the motor exists */
    if (req.motor_number > MAX_INDEX_OF_MOTORS)
    {
        ROS_ERROR("No Motor %d. Max Motor index is %d", req.motor_number, MAX_INDEX_OF_MOTORS);
        return false;
    }

    /* Create UART string and send to serial device */
    /* String: CL${X}\n */
    std::string serial_out_str = "CL" + std::to_string(req.motor_number) + "\n";

    /* Sending serial output. Replace with actuall serial*/
    ROS_INFO("Serial Output: %s", serial_out_str.c_str());
    res.motor_response = serial_out(serial_out_str, true);
    res.motor_success = parseSerialResponse(res.motor_response, true);

  return true;
}

/********************************************************************
 * Implementation [Main]
 * @Args     argc is the number of command-line arguments provided
 * @Args     argv is a pointer to the argument strings
/********************************************************************/
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "motor_con");
    ros::NodeHandle nh("~");

    // From System Manager, get the FD
    //ros::ServiceClient getDeviceName = nh.serviceClient<system_manager::device_manager>("/device_manager/getDeviceFd");
    //std::string sys_name;
    //nh.getParam("system_name", sys_name);
    //system_manager::device_manager srv;
    //srv.request.device = sys_name;
    //getDeviceName.call(srv);
    std::string fd;
    nh.getParam("motor_port", fd);
    setupSerialConnection(fd);

    /* Setup all the Different services/commands which we  can call. Each service does its own error handling */
    ros::ServiceServer setMotorForwardService = nh.advertiseService("setMotorForward", setMotorForward);
    ros::ServiceServer setMotorReverseService = nh.advertiseService("setMotorReverse", setMotorReverse);
    ros::ServiceServer stopAllMotorsService   = nh.advertiseService("stopAllMotors", stopAllMotors);
    ros::ServiceServer stopMotorService       = nh.advertiseService("stopMotors", stopMotor);
    ros::ServiceServer getRPMService          = nh.advertiseService("getRPM", getRPM);
    ros::ServiceServer setPWMService          = nh.advertiseService("setRPM", setPWM);
    ros::ServiceServer calibrateMotorService  = nh.advertiseService("calibrateMotor", calibrateMotor);

    /* Wait for callbacks */
    ros::spin();

    if (serial_conn != nullptr)
    {
        ROS_INFO("Closing connection on port %s", serial_conn->getPort().c_str());
        serial_conn->close();
        delete serial_conn;
    }

    return 0;
}
