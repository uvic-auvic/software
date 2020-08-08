#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ctime>
#include "vision/enable_detection.h"

bool perform_detection = false;
namespace enc = sensor_msgs::image_encodings;

void colour_detection(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImageConstPtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception e)
        {
            ROS_ERROR("Couldn't convert from '%s' to 'bgr8'.", e.what());
            return;
        }

    // Do colour detection here
    // Detect green and/or red then report back to main control gui on what was found


    // Have second service call here which 
}

bool enableDetection(vision::enable_detection::Request &req, vision::enable_detection::Response &res) {
    
    perform_detection = req.enable;
    res.detector_enabled = true;
    return res.detector_enabled;
}


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "colour_detection");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/one/image", 5, colour_detection);
  ros::ServiceServer enableDetector = nh.advertiseService("enableDetector", enableDetection);
  ros::Rate loop_rate(10);

  while (nh.ok()) {
      
      ros::spinOnce();
      loop_rate.sleep();
  }


  return 0;
}
