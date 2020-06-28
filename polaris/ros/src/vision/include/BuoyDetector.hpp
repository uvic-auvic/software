#ifndef BUOYDETECTOR
#define BUOYDETECTOR

#include <stdio.h>
#include "opencv2/opencv.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "Detector.hpp"

class BuoyDetector : public Detector
{
private:
    typedef enum
    {
        Jiangshi,
        Aswang,
        Draugr,
        Vetalas
    } Buoy_t;
    bool found_buoy = false;
    u_int32_t distance_x;
    CameraInput& camera_input;
    Buoy_t buoy;
    u_int8_t min_match_count = 10;
    float ratio_thresh = 0.6f; // ratio for Lowe's ratio test
    float buoy_width = 60.96f; 
    float buoy_height = 123.19f;
    cv::Rect buoy_rect; // rectangle around buoy
    
    struct Detector {
        cv::Mat buoy_img;
        cv::Ptr<cv::xfeatures2d::SIFT> sift;
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;
        cv::Ptr<cv::DescriptorMatcher> matcher;
    };  
    Detector detector;

public:
    BuoyDetector(CameraInput& input, std::string cascade_name);
    ~BuoyDetector();
    bool update();
    cv::Rect GetRect();
};

#endif