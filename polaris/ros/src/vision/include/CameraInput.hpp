#ifndef CAMERAINPUT_HPP
#define CAMERAINPUT_HPP

#include "opencv2/opencv.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"


class CameraInput
{
public:
    CameraInput();
    ~CameraInput() = default;

    bool update();

    const cv::Mat& getFrameFront();
    const cv::Mat& getFrameBottom();
    const cv::Mat& getFrameTop();

private:
    cv::Mat frame_front;
    cv::Mat frame_bottom;
    cv::Mat frame_top;

    cv::VideoCapture input_front;
    cv::VideoCapture input_bottom;
    cv::VideoCapture input_top;
};

#endif