#ifndef DERIVEDDERIVATIVE
#define DERIVEDDERIVATIVE

#include "Detector.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

class DerivedDetector : public Detector
{
public:
    DerivedDetector(CameraInput &input, std::string cascade_name);
    ~DerivedDetector() = default;
    bool update();
    // add functions as needed

private:
    CameraInput &camera_input;

    // add functions/variables as needed
};

#endif