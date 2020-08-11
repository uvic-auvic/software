/***************************************************************
 * @file /include/redgui/qtab.hpp
 * @brief Tab Manager. Take out as much of the implementation
 *        code from MainWindow as possible
 * @date February 2017
/***************************************************************/

/***************************************************************
 * Header Guard
/***************************************************************/
#ifndef VISION_VIDEOSOURCE_H
#define VISION_VIDEOSOURCE_H

/*************************************************************
 * Includes
/*************************************************************/
#include <string>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

/*************************************************************
 * Interface [Control Tab]
/*************************************************************/
class VideoSource {

/*************************************************************
 * VideoSource Interface
/*************************************************************/
public:

    /*
     * VideoSource Constructor doesn't actually do anything
     */
    VideoSource() {}

    /*
     * Sets the video source depending on what kind of source it is
     */
    virtual bool SetSource(const std::string source);

    /*
     * Sets the FPS
     */
    virtual void SetFPS(int fps);

    /*
     * Gets the FPS
     */
    int GetFPS() { return fps; }

    /*
     * Captures the next frame and converts it to a ROS Image format
     */
    virtual cv::Mat capture();

    /*
     * Inits the file which will be written to
     */
    void initOutFile();

    /*
     * Object which writes the video file
     */
    cv::VideoWriter video;
    
private:

    /*
     * FPS of video source output
     */
    int fps = 20;

    /*
     * Checks if the source exists or is attainable
     */
    virtual bool SourceExists(const std::string source);

    /*
     * CV frame of image
     */
    cv::Mat frame;

    /*
     * Capture device
     */
    cv::VideoCapture cap;
}; 


#endif // end of header guard
