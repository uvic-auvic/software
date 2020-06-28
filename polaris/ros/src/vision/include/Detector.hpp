#ifndef DETECTOR
#define DETECTOR

#include "opencv2/opencv.hpp"
#include "CameraInput.hpp"

class Detector
{
public:
    virtual bool update() = 0;

    uint32_t getXFront() const { return distance_x_front; }
    uint32_t getYFront() const { return distance_y_front; }
    uint32_t getZFront() const { return distance_z_front; }
    uint32_t getXBottom() const { return distance_x_bottom; }
    uint32_t getYBottom() const { return distance_y_bottom; }
    uint32_t getZBottom() const { return distance_z_front; }
    uint32_t getXTop() const { return distance_x_top; }
    uint32_t getYTop() const { return distance_y_top; }
    uint32_t getZTop() const { return distance_z_top; }

protected:

    cv::CascadeClassifier cascade;
    
    cv::Rect object_front;
    cv::Rect object_bottom;
    cv::Rect object_top;

    uint32_t distance_x_front = 0;
    uint32_t distance_y_front = 0;
    uint32_t distance_z_front = 0;
    uint32_t distance_x_bottom = 0;
    uint32_t distance_y_bottom = 0;
    uint32_t distance_z_bottom = 0;
    uint32_t distance_x_top = 0;
    uint32_t distance_y_top = 0;
    uint32_t distance_z_top = 0;
};

#endif
