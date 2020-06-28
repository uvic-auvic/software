#ifndef FILTER
#define FILTER

#include "opencv2/opencv.hpp"
#include <cmath>
#include <vector>

class Filter
{
public:
    void updateFilter(std::vector<cv::Rect> locations);
    cv::Rect getBestRect() { return rect; }

private:
    cv::Rect rect;

    struct point_data {
        cv::Rect r;
        uint8_t n;
    };

    std::vector<point_data> points;
    uint8_t iteration = 0;
    const uint16_t radius = 100;
    const uint8_t reset_time = 8;
    
    void updateBestPoint();
    void addPoint(cv::Rect point);
    void removePoint();
};

#endif