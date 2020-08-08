/***************************************************************
 * @file image_sender.cpp
 * @brief The node which provides a video source of some kind
 * @date February 2017
/***************************************************************/

/***************************************************************
 * Includes
/***************************************************************/
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <string>
#include "../include/video_api.h"


/*************************************************************
 * Implementation [Image Sender]
/*************************************************************/
int main (int argc, char ** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    std::string name, fd, pub;
    nh.getParam("name", name);
    nh.getParam("fd", fd);
    nh.getParam("pub", pub);
    image_transport::Publisher publisher = it.advertise(pub, 5);
    VideoSource source;

    if (!source.SetSource(fd)) {
        ROS_ERROR("%s failed to open device on %s", name.c_str(), fd.c_str());
        return -1;
    }

    ros::Rate loop_rate(source.GetFPS());

    while(nh.ok())
    {
        cv::Mat frame = source.capture();
        if (frame.empty()) {
            continue;
        }

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        publisher.publish(msg);
        if (source.video.isOpened()) {
            source.video.write(frame);
        }
        cv::waitKey(1);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
