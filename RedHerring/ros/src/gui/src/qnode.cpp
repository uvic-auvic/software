/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include <ros/network.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <QWidget>
#include <QLabel>
#include <string>
#include <math.h>
#include <sstream>
#include <fstream>

#include "../include/qnode.hpp"


/*****************************************************************************
 * Namespaces
/*****************************************************************************/
using namespace cv;
namespace redgui {

/*****************************************************************************
 * Implementation
/*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init()
{
    ros::init(init_argc, init_argv, "redgui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
    image_transport::ImageTransport it(n);
	// Add your ros communications here.
    //TODO: Uncomment when times comes. Also maybe convert topic method to service calls

    //thrusterSensitivity_publisher = n.advertise<thrusters::Sensitivity>("/GUI/thrusterSensitivity", 4);
    //thrusterValue_subscriber = n.subscribe("/GUI/thrusterValues", 1, &QNode::thrusterValueCb, this);
    //chatter_subscriber = n.subscribe<std_msgs::String>("/GUI/general", 1, &QNode::chatterCb, this);
    //thrusterTemperature_subscriber = n.subscribe("/input/thrusterTemperature", 1, &QNode::temperatureCb, this);
    camera_subscriber = it.subscribe("/camera/one/image", 1, &QNode::imageCb, this);
    camera_subscriber2 = it.subscribe("/camera/two/image", 1, &QNode::imageCb2, this);
    //lightState_publisher = n.advertise<std_msgs::Bool>("/GUI/lights", 1);
    //throttleLockout_publisher = n.advertise<std_msgs::Bool>("/GUI/throttleLockout",1);
    //GUI_ROV_to_AUV_pub_ = n.advertise<std_msgs::Int16>("/GUI/ROVtoAUV", 4);
    //GUI_record_mode_pub_ = n.advertise<std_msgs::Int16>("/GUI/recordingMode", 4);


    /*
     * Configuration File Values
     */
    control_mode.data = 0; // 0: ROV mode, 1: AUV mode, else error -> default to ROV
    videoRecordMode.data = 1;
    forwardInvert_ = false;
    pitchInvert_ = false;
    rollInvert_ = false;
    yawInvert_ = false;
    std::ifstream guiConfigFile;
    std::string fileLocation("~/Github_Projects/RedHerring/ros/src/redgui/include/redgui/redguiConfig.txt");// TODO: Fix later( (std::string(getenv("HOME")) + "/ROV02/src/redgui/include/redgui/redguiConfig.txt") );
    guiConfigFile.open(fileLocation.c_str());
    if(guiConfigFile.is_open())
    {
        std::string line;
        char* lineChar;
        while(getline(guiConfigFile, line)){
            lineChar = (char*)line.c_str();
            line = strtok(lineChar, ";");
            if(line == "_listViewThreshold"){
                    line = strtok(NULL,";");
                    _listViewThreshold = strtol(line.c_str(),0,10);
            }
            else if(line == "forwardSensitivity"){
                    line = strtok(NULL,";");
                    sensitivityData_.forwardSensitivity = strtol(line.c_str(),0,10);
            }
            else if(line == "pitchSensitivity"){
                    line = strtok(NULL,";");
                    sensitivityData_.pitchSensitivity = strtol(line.c_str(),0,10);
            }
            else if(line == "rollSensitivity"){
                    line = strtok(NULL,";");
                    sensitivityData_.rollSensitivity = strtol(line.c_str(),0,10);
            }
            else if(line == "yawSensitivity"){
                    line = strtok(NULL,";");
                    sensitivityData_.yawSensitivity = strtol(line.c_str(),0,10);
            }
            else if(line == "ascentSensitivity"){
                    line = strtok(NULL,";");
                    sensitivityData_.ascentSensitivity = strtol(line.c_str(),0,10);
            }
            else if(line == "ROVtoAUVcontrolMode"){
                line = strtok(NULL,";");
                control_mode.data = strtol(line.c_str(),0,10);
            }
            else
            {
                ROS_INFO("Unknown config input: %s", lineChar);
            }
        }
    }
    else
    {
       ROS_INFO("GUI Config file not found... Using defaults.");
       _listViewThreshold = 50;

       sensitivityData_.forwardSensitivity = 0;
       sensitivityData_.pitchSensitivity = 0;
       sensitivityData_.rollSensitivity = 0;
       sensitivityData_.yawSensitivity = 0;
       sensitivityData_.ascentSensitivity = 0;

    }

    start();
    QNode::sensitivityPublish();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
    _listViewThreshold = 50;

	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;

    //TODO: change back once fixed
    ros::init(remappings,"redgui_remote");

	if ( ! ros::master::check() ) {
		return false;
	}

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
    image_transport::ImageTransport it(n);
	// Add your ros communications here.
    //thrusterSensitivity_publisher = n.advertise<thrusters::Sensitivity>("/GUI/thrusterSensitivity", 10);
    // Commenting out all subscribers because there is nothing to subscribe to
    //thrusterTemperature_subscriber = n.subscribe("/input/thrusterTemperature", 1, &QNode::temperatureCb, this);
    //chatter_subscriber = n.subscribe<std_msgs::String>("/GUI/general", 1, &QNode::chatterCb, this);
    camera_subscriber = it.subscribe("/camera/one/image", 1, &QNode::imageCb, this);
    camera_subscriber2 = it.subscribe("/camera/two/image", 1, &QNode::imageCb2, this);
    // thrusterValue_subscriber = n.subscribe("/GUI/thrusterValues", 10, &QNode::thrusterValueCb, this);
    //lightState_publisher = n.advertise<std_msgs::Bool>("/GUI/lights", 1);
    //throttleLockout_publisher = n.advertise<std_msgs::Bool>("/GUI/throttleLockout",1);
    //GUI_ROV_to_AUV_pub_ = n.advertise<std_msgs::Int16>("/GUI/ROVtoAUV", 4);
    //GUI_record_mode_pub_ = n.advertise<std_msgs::Int16>("/GUI/recordingMode", 4);
    /*
     * Configuration File Values
     */
    control_mode.data = 0; // 0 ROV mode, 1 AUV mode, else error -> default to ROV
    videoRecordMode.data = 1;
    forwardInvert_ = false;
    pitchInvert_ = false;
    rollInvert_ = false;
    yawInvert_ = false;

    std::ifstream guiConfigFile;
    std::string fileLocation("~/Github_Projects/RedHerring/ros/src/redgui/include/redgui/redguiConfig.txt");//( (std::string(getenv("HOME")) + "/ROV02/src/redgui/include/redgui/redguiConfig.txt") );
    guiConfigFile.open(fileLocation.c_str());
    if(guiConfigFile.is_open())
    {
        std::string line;
        char* lineChar;
        while(getline(guiConfigFile, line)){
            lineChar = (char*)line.c_str();
            line = strtok(lineChar, ";");
            if(line == "_listViewThreshold"){
                    line = strtok(NULL,";");
                    _listViewThreshold = strtol(line.c_str(),0,10);
            }
            else if(line == "forwardSensitivity"){
                    line = strtok(NULL,";");
                    sensitivityData_.forwardSensitivity = strtol(line.c_str(),0,10);
            }
            else if(line == "pitchSensitivity"){
                    line = strtok(NULL,";");
                    sensitivityData_.pitchSensitivity = strtol(line.c_str(),0,10);
            }
            else if(line == "rollSensitivity"){
                    line = strtok(NULL,";");
                    sensitivityData_.rollSensitivity = strtol(line.c_str(),0,10);
            }
            else if(line == "yawSensitivity"){
                    line = strtok(NULL,";");
                    sensitivityData_.yawSensitivity = strtol(line.c_str(),0,10);
            }
            else if(line == "ascentSensitivity"){
                    line = strtok(NULL,";");
                    //sensitivityData_.ascentSensitivity = strtol(line.c_str(),0,10);
            }
            else if(line == "ROVtoAUVcontrolMode"){
                line = strtok(NULL,";");
                control_mode.data = strtol(line.c_str(),0,10);
            }
            else
            {
                ROS_INFO("Unknown config input: %s", lineChar);
            }
        }
        ROS_INFO("GUI Config complete.");
    }
    else
    {
       ROS_INFO("GUI Config file not found... Using defaults.");
       _listViewThreshold = 50;

       sensitivityData_.forwardSensitivity = 10;
       sensitivityData_.pitchSensitivity = 10;
       sensitivityData_.rollSensitivity = 10;
       sensitivityData_.yawSensitivity = 10;
       sensitivityData_.ascentSensitivity = 10;

    }

    start();
    QNode::sensitivityPublish();
	return true;
}

void QNode::run() {

    ros::spin();
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
    switch (level) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    if(logging_model.rowCount() > _listViewThreshold){
        logging_model.removeRow((logging_model.rowCount()-1-_listViewThreshold));
    }
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::cvtColor(cv_ptr->image, RGBframe, CV_BGR2RGB);
    img = QImage((const unsigned char*)(RGBframe.data), RGBframe.cols, RGBframe.rows, QImage::Format_RGB888);

    Q_EMIT processedImage(img);
}

void QNode::imageCb2(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::cvtColor(cv_ptr->image, RGBframe2, CV_BGR2RGB);
    img2 = QImage((const unsigned char*)(RGBframe2.data), RGBframe2.cols, RGBframe2.rows, QImage::Format_RGB888);
    Q_EMIT processedImage2(img2);
}

void QNode::chatterCb(const std_msgs::String::ConstPtr& msg){
    if(ros::ok){
        log(Info,msg->data.c_str());
    }
}


void QNode::forwardSensitivity(int value)
{

    if(forwardInvert_ == false)
        sensitivityData_.forwardSensitivity = value;
    else
        sensitivityData_.forwardSensitivity = -1*value;

    QNode::sensitivityPublish();

}

void QNode::pitchSensitivity(int value)
{

    if(pitchInvert_ == false)
        sensitivityData_.pitchSensitivity = value;
    else
        sensitivityData_.pitchSensitivity = -1*value;

    QNode::sensitivityPublish();
}

void QNode::rollSensitivity(int value)
{

    if(rollInvert_ == false)
        sensitivityData_.rollSensitivity = value;
    else
        sensitivityData_.rollSensitivity = -1*value;
    QNode::sensitivityPublish();
    return;

}

void QNode::yawSensitivity(int value)
{

    if(yawInvert_ == false)
        sensitivityData_.yawSensitivity = value;
    else
        sensitivityData_.yawSensitivity = -1*value;

    QNode::sensitivityPublish();
}

void QNode::ascentSensitivity(int value)
{
    sensitivityData_.ascentSensitivity = value;
    QNode::sensitivityPublish();
}

void QNode::sensitivityPublish()
{
    //thrusterSensitivity_publisher.publish(sensitivityData_);
}

void QNode::thrusterValueCb(thrusterValues thrustValues)
{

    Q_EMIT thrusterForwardSignal((float)thrustValues.forwardThruster );
    Q_EMIT thrusterPitchSignal((float)thrustValues.pitchThruster);
    Q_EMIT thrusterRollSignal((float)thrustValues.rollThruster);
    Q_EMIT thrusterYawSignal((float)thrustValues.yawThruster);
    Q_EMIT thrusterAscentSignal((float)thrustValues.ascent);
    return;

}

void QNode::temperatureCb(thrusterTemperatures temperatures)
{
    Q_EMIT UCDieTemperatureSignal(temperatures.UCDieTemperature);
    Q_EMIT OILTemperatureSignal(temperatures.oilTemperature);
}


void QNode::lightChange(bool lightState)
{
    std_msgs::Bool boolLightState;
    boolLightState.data = lightState;
    //lightState_publisher.publish(boolLightState);
    return;
}


void QNode::throttleLockoutChange(bool lockoutState)
{
    std_msgs::Bool boolLockoutState;
    boolLockoutState.data = lockoutState;
    //throttleLockout_publisher.publish(boolLockoutState);
    QNode::sensitivityPublish();
    return;
}

int QNode::sensitivityData(std::string whichData)
{
    if(whichData == "forwardSensitivity")
        return (int)sensitivityData_.forwardSensitivity;
    else if(whichData == "pitchSensitivity")
        return (int)sensitivityData_.pitchSensitivity;
    else if(whichData == "rollSensitivity")
        return (int)sensitivityData_.rollSensitivity;
    else if(whichData == "yawSensitivity")
        return (int)sensitivityData_.yawSensitivity;
    else if(whichData == "ascentSensitivity")
        return (int)sensitivityData_.ascentSensitivity;
    else
        return 0;
}
void QNode::forwardInvertChange(bool invertState)
{
    forwardInvert_ = invertState;
    sensitivityData_.forwardSensitivity = -1*sensitivityData_.forwardSensitivity;
    QNode::sensitivityPublish();
    return;
}

void QNode::rollInvertChange(bool invertState)
{
    rollInvert_ = invertState;
    sensitivityData_.rollSensitivity = -1*sensitivityData_.rollSensitivity;
    QNode::sensitivityPublish();
    return;
}

void QNode::pitchInvertChange(bool invertState)
{
    pitchInvert_ = invertState;
    sensitivityData_.pitchSensitivity = -1*sensitivityData_.pitchSensitivity;
    QNode::sensitivityPublish();
    return;
}

void QNode::yawInvertChange(bool invertState)
{
    yawInvert_ = invertState;
    sensitivityData_.yawSensitivity = -1*sensitivityData_.yawSensitivity;
    QNode::sensitivityPublish();
    return;
}



void QNode::updateControlMode(const ControlMode control_mode)
{
    switch (control_mode)
    {

    case ROV:
        ROS_INFO("Toggling ROV Mode");
        this->control_mode.data = ROV;
        Q_EMIT enableControls(true);
        break;

    case AUV:
        ROS_INFO("Toggling AUV Mode");
        this->control_mode.data = AUV;
        Q_EMIT enableControls(false);
        break;

    default:
        ROS_WARN("Invalid Mode Input. Defaulting to ROV Mode.");
        this->control_mode.data = ROV;
        Q_EMIT enableControls(true);
        break;

    }

    //GUI_ROV_to_AUV_pub_.publish(ROVtoAUVcontrolMode);
    return;
}

void QNode::updateVideoRecordMode(int value)
{
    /* if value == 0 recording will begin, for all other values, recording is stopped */
    videoRecordMode.data = value;
    if(value == 0)
        ROS_INFO("Video Record Mode: recording.");
    else
        ROS_INFO("Video Record Mode: stopped.");
      GUI_record_mode_pub_.publish(videoRecordMode);
    return;
}

int QNode::whichControlMode()
{
    return (int) control_mode.data;
}

int QNode::whichVideoRecordMode()
{
    return (int) videoRecordMode.data;
}


}  // namespace redgui
