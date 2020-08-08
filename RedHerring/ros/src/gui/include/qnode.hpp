/**
 * @file /include/redgui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef REDGUI_QNODE_HPP
#define REDGUI_QNODE_HPP

/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include <QWidget>
#include <QThread>
#include <QStringListModel>
#include <QLabel>
#include <QApplication>
#include <QStatusBar>
#include <QImage>
#include <QKeyEvent>
#include <QMenu>
#include <QMainWindow>
#include <QDebug>
#include <QFileDialog>
#include <QMenuBar>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include "std_msgs/Int32.h"


/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace redgui {

/*****************************************************************************
 * Enums
*****************************************************************************/
    enum LogLevel {
        Debug,
        Info,
        Warn,
        Error,
        Fatal
     };

    enum ControlMode
    {
        ROV,          // = 0
        AUV           // = 1
    };

/*****************************************************************************
 * Structs
 * **************************************************************************/
struct sensitivityData {
    float forwardSensitivity;
    float pitchSensitivity;
    float rollSensitivity;
    float yawSensitivity;
    float ascentSensitivity;
    float forwardThrusterBar;
};

struct thrusterValues {
    float forwardThruster;
    float pitchThruster;
    float rollThruster;
    float yawThruster;
    float ascent;
};

struct thrusterTemperatures {
    float UCDieTemperature;
    float oilTemperature;
};

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	QStringListModel* loggingModel() { return &logging_model; }
    int  whichControlMode();
    int  whichVideoRecordMode();
    int  sensitivityData(std::string whichData);
	void log( const LogLevel &level, const std::string &msg);
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void imageCb2(const sensor_msgs::ImageConstPtr& msg);
    void chatterCb(const std_msgs::String::ConstPtr& msg);
    void forwardSensitivity(int value);
    void pitchSensitivity(int value);
    void rollSensitivity(int value);
    void yawSensitivity(int value);
    void ascentSensitivity(int value);
    void sensitivityPublish();
    void thrusterValueCb(thrusterValues thrustValues);
    void temperatureCb(thrusterTemperatures temperatures);
    void lightChange(bool lightState);
    void throttleLockoutChange(bool lockoutState);
    void forwardInvertChange(bool invertState);
    void rollInvertChange(bool invertState);
    void pitchInvertChange(bool invertState);
    void yawInvertChange(bool invertState);
    void updateControlMode(const ControlMode control_mode);
    void updateVideoRecordMode(int value);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void processedImage(const QImage &image);
    void processedImage2(const QImage &image);
    void valueChanged(float value);
    void thrusterForwardSignal(float thrustValue);
    void thrusterPitchSignal(float thrustValue);
    void thrusterRollSignal(float thrustValue);
    void thrusterYawSignal(float thrustValue);
    void UCDieTemperatureSignal(float value);
    void OILTemperatureSignal(float value);
    void thrusterAscentSignal(float ascentCommand);
    void enableControls(bool enable);

private:
	int init_argc;
    int _listViewThreshold;
	char** init_argv;
	ros::Publisher chatter_publisher;
    ros::Publisher thrusterSensitivity_publisher;
    ros::Publisher throttleLockout_publisher;
    ros::Publisher lightState_publisher;
    ros::Publisher GUI_ROV_to_AUV_pub_;
    ros::Publisher GUI_record_mode_pub_;
    ros::Subscriber thrusterValue_subscriber;
    ros::Subscriber thrusterTemperature_subscriber;
    ros::Subscriber chatter_subscriber;
    image_transport::Subscriber camera_subscriber, camera_subscriber2;
    QStringListModel logging_model;
    QImage img, img2;
    cv::Mat RGBframe, RGBframe2;

    struct sensitivityData sensitivityData_;
    struct thrusterValues thrustValues;
    struct thrusterTemperatures temperatures;

    std_msgs::Int16 control_mode, videoRecordMode;
    bool forwardInvert_, rollInvert_, pitchInvert_, yawInvert_;

};

}  // namespace redgui

#endif /* redgui_QNODE_HPP_ */
