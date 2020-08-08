#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "vision/enable_detection.h"
#include <ctime>

int count = 0;
std::clock_t start;

class enableProcessing
{
public:
    bool enable = false;
    bool changeState = false;
    bool changeStates(vision::enable_detection::Request &req,
                      vision::enable_detection::Response &res)
    {
        this->changeState = true;
        this->enable = req.enable;
        return true;
    }
};

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
      cv::imshow("Webcam", cv_bridge::toCvShare(msg, "bgr8")->image);
      cv::waitKey(30);
    }
    catch (cv_bridge::Exception e)
    {
      ROS_ERROR("Couldn't convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_reciever");
    ros::NodeHandle nh("~");
    cv::namedWindow("Webcam");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/one/image", 5, imageCallback);
    enableProcessing enabler;
    ros::ServiceServer MoveForward = nh.advertiseService("enableProcessing", &enableProcessing::changeStates, &enabler);

    while (ros::ok())
    {
        if (enabler.changeState)
        {
            enabler.changeState = false;
            if (enabler.enable) 
            {
                cv::namedWindow("Webcam");
                cv::startWindowThread();
                sub = it.subscribe("/camera/one/image", 5, imageCallback);
            } else
            {
                sub.shutdown();
                cv::destroyWindow("Webcam");
            }
        } 

        ros::spinOnce();
    }

    cv::destroyWindow("Webcam");

    return 0;
}
