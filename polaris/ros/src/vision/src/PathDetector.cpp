#include "PathDetector.hpp"
#include "Distance.hpp"

#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#define LOW_HUE         10
#define HIGH_HUE        30
#define LOW_SAT         150
#define HIGH_SAT        255
#define LOW_VAL         100
#define HIGH_VAL        255

PathDetector::PathDetector(CameraInput& input, std::string cascade_name) : camera_input(input)
{
    if(cascade_name != "") cascade.load(cascade_name);
}

PathDetector::~PathDetector()
{}

bool PathDetector::update()
{
    cv::Mat frame_hsv;
    cv::Mat frame_thresh;
    std::vector<std::vector<cv::Point>> contours;

    cv::cvtColor(camera_input.getFrameFront(), frame_hsv, CV_BGR2HSV);
    cv::inRange(frame_hsv, cv::Scalar(LOW_HUE, LOW_SAT, LOW_VAL), cv::Scalar(HIGH_HUE, HIGH_SAT, HIGH_VAL), frame_thresh);

    // Test to see if following are beneficial
    cv::GaussianBlur(frame_thresh, frame_thresh, cv::Size(3,3), 0);
    cv::dilate(frame_thresh, frame_thresh, 0);
    cv::erode(frame_thresh, frame_thresh, 0);

    cv::findContours(frame_thresh, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    // finds the lasgest area of color detected
    int index_largest_area = 0;
    int size_largest_area = contours[0].size();
    if(contours.size() > 1) {
        for(int i=0; i<contours.size(); ++i) {
            if(size_largest_area < contours[i].size()) {
                size_largest_area = contours.size();
                index_largest_area = i;
            }
        }
    }

    // Not sure if this will work, gets a rotated rect around largest area found
    // and converts it to a rect. Conversion method may need to be changed
    object_front = cv::boundingRect(contours[index_largest_area]);


    // object_front = cv::boundingRect(contours);

    distance_x_front = Distance::getDistanceX(object_front, 15.24, camera_input.getFrameFront());
    distance_y_front = Distance::getDistanceY(object_front, 120.0, camera_input.getFrameFront());
    distance_z_front = Distance::getDistanceZ(object_front, 15.24, 1); 
}