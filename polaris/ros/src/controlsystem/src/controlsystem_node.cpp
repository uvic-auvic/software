#include <ros/ros.h>
#include <ros/console.h>
#include <boost/geometry.hpp>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <string>
#include <stack>
#include <stdexcept>

#include "statemachine.hpp"
#include "statussystem.hpp"

/*!
 * This class is the "Brains" of the operation.
 * It controls state switching, aggregating sensor
 * information, and publishing information for sub
 * systems.
 */
class ControlSystem {
public:
  using point = boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian>;

  enum struct UpdateStatus : int {
	  OKAY = 0,
	  MINOR_ERROR = 1,
	  MAJOR_ERROR = 2,
	  FATAL_ERROR = 3,
  };

  /*!
   * Converts a UpdateStatus enum to a human-readable format.
   * @param x status to convert.
   * @return human-readable equivalent of status.
   */
  static std::string to_string(const UpdateStatus x) {
		switch (x) {
			case UpdateStatus::OKAY:
				return "OKAY";
			case UpdateStatus::MINOR_ERROR:
				return "MINOR_ERROR";
			case UpdateStatus::MAJOR_ERROR:
				return "MAJOR_ERROR";
			case UpdateStatus::FATAL_ERROR:
				return "FATAL_ERROR";
			default:
				return "UNDEFINED";
		}
	}

private:
  ros::NodeHandle& nodeHandle_;

  StatusSystem statusSystem_;

  StateMachine stateMachine_;

  double _depth_update()
  {
  	return 0.0;
  }

  UpdateStatus _run_updates()
  {
		statusSystem_.depth = _depth_update();
  	return UpdateStatus::OKAY;
  }

public:
	ControlSystem() = delete;
  explicit ControlSystem(ros::NodeHandle& nh)
    : nodeHandle_(nh), statusSystem_{}, stateMachine_{}
  {

    XmlRpc::XmlRpcValue state_params;
    nodeHandle_.getParam("states", state_params);
    stateMachine_ = StateMachine(state_params);

    stateMachine_.print_debug();
  }

  int operator()() noexcept
  {
  	StateMachine::StepResult step_result = StateMachine::StepResult::CONTINUE;
    UpdateStatus update_status = UpdateStatus::OKAY;
  	ros::Rate r(2);
    while(step_result == StateMachine::StepResult::CONTINUE)
      {
    	  step_result = stateMachine_();
    	  ROS_INFO_STREAM("Stepping: " << StateMachine::to_string(step_result));
    	  update_status = _run_updates();
    	  ROS_INFO_STREAM("Updating:" << to_string(update_status));
        ros::spinOnce();
        r.sleep();
      }
    return static_cast<int>(step_result) + static_cast<int>(update_status);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controlsystem");
  ros::NodeHandle nh("~");

  ControlSystem control_system(nh);

  int return_code = control_system();

  ros::shutdown();
  return return_code;
}
