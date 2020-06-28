#ifndef DISTANCE
#define DISTANCE

#define FRONT_FOCAL 380.65
#define TOP_FOCAL 481.81
#define BOTTOM_FOCAL 0

#include "opencv2/opencv.hpp"

class Distance
{
public:
    static uint32_t getDistanceX(cv::Rect object, float realWidth, cv::Mat frame);
    static uint32_t getDistanceY(cv::Rect object, float realHieght, cv::Mat frame);
    static uint32_t getDistanceZ(cv::Rect object, float realWidth, uint8_t camera);

private:
    uint32_t z_distance = 0;
    uint32_t x_distance = 0;
    uint32_t y_distance = 0;

};

#endif