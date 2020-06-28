#include <ros/ros.h>

#include "vision/vector.h"
#include "vision/change_detection.h"

#include "EnableDetector.hpp"
#include "CameraInput.hpp"
#include "BuoyDetector.hpp"
#include "GateDetector.hpp"
#include "PathDetector.hpp"

// VisionSystem::EnabledDetector::NONE

class VisionSystem
{
private:
    // Ros fun stuff
    ros::NodeHandle nh_;

    ros::Publisher pub_;
    ros::ServiceServer changeDetection_;

    vision::vector msg_;

    // Image capturing and detection systems.
    CameraInput cameraInput_;
    GateDetector gateDetector_;
    BuoyDetector buoyDetector_;
    PathDetector pathDetector_;

    // Detectors that are Enabled
    EnabledDetector enabledDetectors_;

public:

	/*!
	 * This is the service for changing the detection type of the detection system.
	 * @param request the requested detection type.
	 * @param response unused/empty.
	 * @return true if success or false if failure.
	 */
	bool changeDetectionCallback(vision::change_detection::Request& request, vision::change_detection::Response& response)
	{
		enabledDetectors_ = static_cast<EnabledDetector>(request.enabled_type);
		return true;
	}

    VisionSystem(ros::NodeHandle& nh) : 
        nh_(nh), 
        cameraInput_(),
        gateDetector_(cameraInput_, "../cascades/GateCascades.xml"),
        buoyDetector_(cameraInput_, ""),
        pathDetector_(cameraInput_, ""),
        enabledDetectors_(EnabledDetector::NONE)
    {
        pub_ = nh.advertise<vision::vector>("/vision/vector", 1);
        changeDetection_ = nh.advertiseService("/vision/change_detection", &VisionSystem::changeDetectionCallback, this);
    }

    /* THIS IS THE MAIN LOOP OF THE VISION SYSTEM */
    int operator()()
    {
        int status = 0;
        
        ros::Rate r(10); // Maybe Faster
        while(ros::ok() && !status)
        {
            if(cameraInput_.update()) 
            {
                switch (enabledDetectors_)
                {
                case EnabledDetector::GATE:
                    gateDetector_.update();
                    msg_.x = gateDetector_.getXFront();
                    msg_.y = gateDetector_.getYFront();
                    msg_.z = gateDetector_.getZFront();
                    break;
                case EnabledDetector::BUOY:
                    buoyDetector_.update();
                    msg_.x = buoyDetector_.getXFront();
                    msg_.y = buoyDetector_.getYFront();
                    msg_.z = buoyDetector_.getZFront();
                    break;
                case EnabledDetector::PATH:
                    pathDetector_.update();
                    msg_.x = pathDetector_.getXFront();
                    msg_.y = pathDetector_.getYFront();
                    msg_.z = pathDetector_.getZFront();
                    break;
                case EnabledDetector::NONE:
                default:
                    msg_.x = 0;
                    msg_.y = 0;
                    msg_.z = 0;
                    break;
                }
            }

            pub_.publish(msg_);
            ros::spinOnce();
            r.sleep();
        }
        
        return status;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");
    ros::NodeHandle nh("~");

    VisionSystem visionSystem(nh);

    int status = visionSystem();

    ros::shutdown();
    return status;
}