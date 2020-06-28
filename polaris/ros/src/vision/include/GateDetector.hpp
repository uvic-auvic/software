#ifndef GATEDETECTOR
#define GATEDETECTOR

#include "Detector.hpp"

class GateDetector : public Detector
{
public:
    GateDetector(CameraInput& input, std::string cascade_name);
    ~GateDetector();

    bool update();

private:
    cv::Point findGateDivider(cv::Mat frame);
    cv::Point avgPoint(std::vector<cv::Point2f> list);

    CameraInput& camera_input;
};

#endif
