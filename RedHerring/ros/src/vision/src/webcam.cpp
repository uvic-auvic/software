/***************************************************************
 * @file /src/qtab.cpp
 * @brief Tab Manager. Take out as much of the implementation
 *        code from MainWindow as possible
 * @date February 2017
/***************************************************************/

/*************************************************************
 * Includes
/*************************************************************/
#include <string>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <ros/ros.h>
#include "../include/video_api.h"

/*************************************************************
 * Implementation [Control Tab]
/*************************************************************/

/*************************************************************
 * @Name     ReadSettings
 * @Args     filename
 * @function reads and stores settings from the file passed
 *************************************************************/
bool VideoSource::SetSource(const std::string source) {

    /* The file descripter will be something like /dev/video0, but we want the last char 0 as an int */
    char last_char = source.back();
    int device_index = last_char - '0';

    // Not very type-safe/templated, but whatever
    cap = cv::VideoCapture(device_index);

    return cap.isOpened();
}

/*************************************************************
 * @Name     ReadSettings
 * @Args     filename
 * @function reads and stores settings from the file passed
 *************************************************************/
void VideoSource::SetFPS(int fps = 15) {
    this->fps = fps;
}

/*************************************************************
 * @Name     ReadSettings
 * @Args     filename
 * @function reads and stores settings from the file passed
 ************************************************************/
cv::Mat VideoSource::capture() {
    cap >> frame;

    return frame;
}

/*************************************************************
 * @Name     ReadSettings
 * @Args     filename
 * @function reads and stores settings from the file passed
 *************************************************************/
bool VideoSource::SourceExists(const std::string source) {
    return true;
}

/*************************************************************
 * @Name     ReadSettings
 * @Args     filename
 * @function reads and stores settings from the file passed
 *************************************************************/
void VideoSource::initOutFile() {
    int width = this->cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int height = this->cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    cv::Size size = cv::Size(width, height);
    int codec = this->cap.get(CV_CAP_PROP_FOURCC);
    this->video = cv::VideoWriter("~/Documents/test.avi", codec, this->fps, size, true);
}


