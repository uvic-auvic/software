//
// Created by avlec on 27/07/19.
//

#include <ros/ros.h>
#include "controllers.hpp"

// Message Files
#include "navigation/nav.h"
#include "navigation/depth_info.h"
#include "peripherals/imu.h"
#include "peripherals/powerboard.h"
// Service Files
#include "navigation/full_stop.h"
#include "navigation/nav_request.h"
#include "navigation/control_en.h"
#include "peripherals/avg_data.h"

class NavigationSystem {

private:
	// Ros stuffs
	ros::NodeHandle& nodeHandle_;

	ros::Publisher heading_;
	ros::Publisher depth_;
	ros::Subscriber imuData_;
	ros::Subscriber powerBoardData_;
	ros::ServiceServer setHeading_;
	ros::ServiceServer fullStop_;
	ros::ServiceServer controlEnable_;
	ros::ServiceServer calibrateDepth_;

	ros::Rate loopRate_;

	// Functionality related stuff.
	navigation::control_en::Request currentControlEnRequest_;
	navigation::nav_request::Request currentControlRequest_;
	peripherals::imu::ConstPtr currentImuData_;

	// Depth
	double currentDepth_, surfacePressure_;
	bool depthCalibrated_;

	// Position and velocity controllers
	position_controller angular_pos_p;
	position_controller angular_pos_r;
	velocity_controller angular_vel_yw;
	velocity_controller linear_vel_z;

public:
	/*!
	 * @brief Callback for fetching the most recent navigation request.
	 * @param request information to set as the current heading.
	 * @param response not used.
	 * @return always true.
	 */
	bool setHeadingCallback(navigation::nav_request::Request& request, navigation::nav_request::Response& response)
	{
		this->currentControlRequest_ = request;
		return true;
	}

	/*!
	 * @brief Callback for service to stop submarine movement.
	 * @param request empty.
	 * @param response not used.
	 * @return false if depth not calibrated, true otherwise.
	 */
	bool fullStopCallback(navigation::full_stop::Request& request, navigation::full_stop::Response& response)
	{
		// Ensure depth has been calibrated, because this callback needs to know depth.
		if(!depthCalibrated_)
		{
			ROS_WARN("Depth must be calibrated before issuing commands to the navigation system.");
			return false;
		}
		this->currentControlRequest_.depth = currentDepth_;
		this->currentControlRequest_.yaw_rate = 0.0;
		this->currentControlRequest_.forwards_velocity = 0.0;
		this->currentControlRequest_.sideways_velocity = 0.0;
		return true;
	}

	/*!
 	 * @brief Callback for service to enable different components of the control system.
 	 * @param req Service enable request.
 	 * @param res Response to enabled service request.
 	 */
	bool controlEnableCallback(navigation::control_en::Request& request, navigation::control_en::Response& response)
	{
		this->currentControlEnRequest_ = request;

		// Reset any control systems being disabled
		if(!currentControlEnRequest_.vel_z_enable)
		{
			linear_vel_z.reset();
		}
		if(!currentControlEnRequest_.pitch_enable)
		{
			angular_pos_p.reset();
		}
		if(!currentControlEnRequest_.roll_enable)
		{
			angular_pos_r.reset();
		}
		if(!currentControlEnRequest_.yaw_enable)
		{
			angular_vel_yw.reset();
		}
		return true;
	}

	/*!
	 * @brief Service callback to calibrate the surface depth.
	 * @param req Service request message.
	 * @param res Service response message.
	 * @return true upon success, false otherwise.
	 */
	bool calibrateDepthCallback(peripherals::avg_data::Request& request, peripherals::avg_data::Response& response)
	{
		peripherals::avg_data srv;
		srv.request = request;
		ros::ServiceClient external_pressure = nodeHandle_.serviceClient<peripherals::avg_data>("/power_board/AverageExtPressure");

		if(!external_pressure.call(srv))
		{
			ROS_ERROR("Failed to acquire external pressure data during depth calibration.");
			return false;
    }

    response = srv.response;

    // Update surface pressure with the average external pressure
    surfacePressure_ = srv.response.avg_data;
    depthCalibrated_ = true;
    return true;
	}

	void updateDepth(navigation::depth_info& message)
	{
		if(!depthCalibrated_)
		{
			ROS_WARN("Depth must be calibrated before issuing commands to the navigation system.");
			return;
		}
		message.desired_depth = currentControlRequest_.depth;
		message.current_depth = currentDepth_;
	}

	// NOTE: depth sensor needs to be calibrated.
	void updateHeading(navigation::nav& message)
	{
		if(!depthCalibrated_)
		{
			ROS_WARN("Depth must be calibrated before issuing commands to the navigation system.");
			return;
		}

		if(currentControlEnRequest_.vel_x_enable)
		{
			message.direction.x = currentControlRequest_.forwards_velocity;
		}
		if(currentControlEnRequest_.vel_y_enable)
		{
			message.direction.y = currentControlRequest_.sideways_velocity;
		}
		if(currentControlEnRequest_.vel_z_enable)
		{
			message.direction.z = linear_vel_z.calculate(currentControlRequest_.depth, currentDepth_);
		}
		if(currentControlEnRequest_.pitch_enable)
		{
			message.orientation.pitch = angular_pos_p.calculate(0, currentImuData_->euler_angles.pitch, currentImuData_->compensated_angular_rate.y);
		}
		if(currentControlEnRequest_.roll_enable)
		{
			message.orientation.roll = angular_pos_r.calculate(0, currentImuData_->euler_angles.roll, currentImuData_->compensated_angular_rate.x);
		}
		if(currentControlEnRequest_.yaw_enable)
		{
			message.orientation.yaw = angular_vel_yw.calculate(currentControlRequest_.yaw_rate, currentImuData_->angular_rate.z);
		}
	}

	/*!
	 * This callback is used to fetch the latest imu information.
	 */
	void imuDataUpdateCallback(const peripherals::imu::ConstPtr& message)
	{
		this->currentImuData_ = message;
	}

	/*!
	 * This callback is used to fetch the latest depth information from the powerboard.
	 */
	void powerBoardDataUpdateCallback(const peripherals::powerboard::ConstPtr& message)
	{
		constexpr double mul = 1 / (997 * 9.81); // depth = pressure * (1 / (density * gravity))
		this->currentDepth_ = (message->external_pressure -  this->surfacePressure_) * mul;
	}

	// TODO change loop rate to use parameter provided.
	NavigationSystem(ros::NodeHandle& nh)
	: nodeHandle_(nh), loopRate_(10), currentDepth_(0), surfacePressure_(1E5), depthCalibrated_(false)
	{
		// This is replacing /nav/velocity_vectors
		heading_ = nh.advertise<navigation::nav>("/navigation/heading", 1);
		// This is replacing /nav/depth_control_info
		depth_ = nh.advertise<navigation::depth_info>("/navigation/depth", 1);
		// This service is used for setting headings
		// navigation::nav_request
		setHeading_ = nh.advertiseService("/navigation/set_heading", &NavigationSystem::setHeadingCallback, this);
		// This service is used for bringing the submarine to a stand still.
		// navigation::full_stop
		fullStop_ = nh.advertiseService("/navigation/full_stop", &NavigationSystem::fullStopCallback, this);
		// This service is used for turning on/off different control parameters.
		controlEnable_ = nh.advertiseService("/navigation/control_enable", &NavigationSystem::controlEnableCallback, this);
		// This service is used to calibrate surface depth.
		calibrateDepth_ = nh.advertiseService("/navigation/calibrate_depth", &NavigationSystem::calibrateDepthCallback, this);

		// peripherals::imu
		imuData_ = nh.subscribe("/imu/imu_sensor", 1, &NavigationSystem::imuDataUpdateCallback, this);
		// peripherals::powerboard
		powerBoardData_ = nh.subscribe("/power_board/power_board_data", 1, &NavigationSystem::powerBoardDataUpdateCallback, this);

		double loop_rate, min_lin_vel, max_lin_vel;
		double min_angl_vel, max_angl_vel, min_angl_pos, max_angl_pos;

		/*! \todo Default values for parameters that are expected to be
			provided in the launch file in the event they're not provided. */
		nh.getParam("loop_rate", loop_rate);
		nh.getParam("min_linear_vel", min_lin_vel);
		nh.getParam("max_linear_vel", max_lin_vel);
		nh.getParam("min_angular_vel", min_angl_vel);
		nh.getParam("max_angular_vel", max_angl_vel);
		nh.getParam("min_angular_pos", min_angl_pos);
		nh.getParam("max_angular_pos", max_angl_pos);
		double dt = 10.0 / loop_rate;

		// Reassign loop rate
		loopRate_ = ros::Rate(loop_rate);

		// Veloctiy Z Control System
		double Kp_vel_z, Ki_vel_z;
		nh.getParam("Kp_vel_z", Kp_vel_z);
		nh.getParam("Ki_vel_z", Ki_vel_z);

		// Position Pitch Control System
		double Kp_pos_p, Ki_pos_p, Kp_vel_p, Ki_vel_p;
		nh.getParam("Kp_pos_p", Kp_pos_p);
		nh.getParam("Ki_pos_p", Ki_pos_p);
		nh.getParam("Kp_vel_p", Kp_vel_p);
		nh.getParam("Ki_vel_p", Ki_vel_p);

		// Position Roll Control System
		double Kp_pos_r, Ki_pos_r, Kp_vel_r, Ki_vel_r;
		nh.getParam("Kp_pos_r", Kp_pos_r);
		nh.getParam("Ki_pos_r", Ki_pos_r);
		nh.getParam("Kp_vel_r", Kp_vel_r);
		nh.getParam("Ki_vel_r", Ki_vel_r);

		// Velocity Yaw Control System
		double Kp_vel_yw, Ki_vel_yw;
		nh.getParam("Kp_vel_yw", Kp_vel_yw);
		nh.getParam("Ki_vel_yw", Ki_vel_yw);

		// Create angular position controller for pitch
		angular_pos_p = position_controller(min_angl_vel, max_angl_vel,
						min_angl_pos, max_angl_pos,
						dt, Kp_pos_p,	Ki_pos_p, Kp_vel_p, Ki_vel_p);

		// Create angular position controller for roll
		angular_pos_r = position_controller(min_angl_vel, max_angl_vel,
						min_angl_pos, max_angl_pos,dt,
						Kp_pos_r, Ki_pos_r, Kp_vel_r, Ki_vel_r);

		// Create angular velocity controller for yaw
		angular_vel_yw = velocity_controller(min_angl_vel, max_angl_vel,
						dt, Kp_vel_yw, Ki_vel_yw);

		// Create linear velocity controller for the vertical direction.
		linear_vel_z = velocity_controller(min_lin_vel, max_lin_vel,
						dt, Kp_vel_z, Ki_vel_z);
	}

	~NavigationSystem() = default;

	int operator()()
	{
		int status = 0;
		while(ros::ok())
		{
			// Update heading.
			navigation::nav heading_message;
			updateHeading(heading_message);
			heading_.publish(heading_message);

			// Update depth.
			navigation::depth_info depth_message;
			updateDepth(depth_message);
			depth_.publish(heading_message);

			ros::spinOnce();
			loopRate_.sleep();

			if (status) {
				break;
			}
		}
		return status;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "navigationsystem");
	ros::NodeHandle nh("~");

	NavigationSystem navigationSystem(nh);

}