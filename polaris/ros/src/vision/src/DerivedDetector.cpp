#include "DerivedDetector.hpp"



//-------------------------------------------------------------------------------------------------
//
// THIS IS A TEMPLATE
//
//-------------------------------------------------------------------------------------------------

DerivedDetector::DerivedDetector(CameraInput &input, std::string cascade_name) : camera_input(input)
{
    cascade.load(cascade_name);
}

bool DerivedDetector::update()
{
    cv::Mat frame_gray;
    cv::cvtColor(camera_input.getFrameFront(), frame_gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(frame_gray, frame_gray);

    // Detects objects and stores their location in locations
    std::vector<cv::Rect> locations;
    //cascade.detectMultiScale(frame_gray, locations, 1.1, 2, 0|cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
    if(!locations.empty()) {
        
        return true;
    }
    
    return true;
}