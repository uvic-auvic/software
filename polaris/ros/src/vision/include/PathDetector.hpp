#ifndef PATHDETECTOR
#define PATHDETECTOR

#include "Detector.hpp"

class PathDetector : public Detector
{
public:
    PathDetector(CameraInput& input, std::string cascade_name);
    ~PathDetector();

    bool update();

private:
    CameraInput& camera_input;
};

#endif