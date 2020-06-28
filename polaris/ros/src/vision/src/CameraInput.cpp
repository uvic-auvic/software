#include "CameraInput.hpp"
//#include <type_traits>

CameraInput::CameraInput() : input_front(1), input_bottom(2), input_top(0)
{}

bool CameraInput::update()
{
    if(!input_front.read(frame_front)) {
        return false;
    }
    if(!input_bottom.read(frame_bottom)) {
        return false;
    }
    if(!input_top.read(frame_top)) {
        return false;
    }
    return true;
}

const cv::Mat& CameraInput::getFrameFront() { return frame_front; }

const cv::Mat& CameraInput::getFrameBottom() { return frame_bottom; }

const cv::Mat& CameraInput::getFrameTop() { return frame_top; }