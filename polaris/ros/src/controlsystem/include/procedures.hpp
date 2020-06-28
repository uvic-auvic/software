#ifndef PROCEDURES_HPP
#define PROCEDURES_HPP

/*
  This class provides definitions of the different
  functors that are responsible for the various movement
  and feedback patterns that polaris must execute.
 */

#include "navigation/full_stop.h"
#include "navigation/nav.h"
#include "navigation/depth_info.h"
#include "navigation/nav_request.h"
#include "vision/vector.h"
#include "vision/change_detection.h"

namespace procedures {
class Procedure {
  public:

    // This is used to communicate what the return code means
    // to the caller.
    enum class ReturnCode : int {
      FATAL = -2,
      ERROR = -1,
      CONTINUE = 0,
      NEXT = 1
    };

    // Initialize class members and load the movement
    // configuration.
    Procedure() = default;

    // Member for making copies of functors.
    virtual Procedure* clone() const
	  {
		  return new Procedure(*this);
	  }

		// Functors require access to sensors.
		// And mechanism to control submarine.

    // Invoking base operator() will result in FATAL
    // return code.
    virtual ReturnCode operator()() {
			// Sensors to measure actual
			// Compare actual to goal.
			// HMMM this sounds like a PID controller...
			// Could this be provided for convenience in the base
			// class? or should it be left up to the derived?

			// Adjust movement based on previous computation.

			// Based on computation return code to tell StateMachine
			// what the state should be doing.

			return ReturnCode::FATAL;
    };


		// This function is responsible for preparing
    // the sub and functor for use.
		virtual void prep() {
		}

		// This function is repsonsible for unpreparing
    // the sub and functor for use.
		virtual void unprep() {
		}
};

  // GateLocateProcedure updates the navigation heading from the vector provided by the vision GateDector class
class GateLocateProcedure: public Procedure {

		ros::NodeHandle n;
    ros::ServiceClient full_stop_srv;
		ros::ServiceClient set_heading;
    ros::ServiceClient gate_detection;
		ros::Subscriber heading;
    ros::Subscriber vision_vector;
    navigation::nav current_heading;
		navigation::nav nav_heading;

    bool yaw_recorded;
    double start_yaw;
		bool has_gate_vector;
    bool has_heading;

    uint16_t x, y, z;

	public:

    void vectorUpdateCallback(const vision::vector::ConstPtr& message)
    {

      if (message) {
        x = message->x;
        y = message->y;
        z = message->z;
        has_gate_vector = true;
      }
    }

    void updateHeadingCallback(navigation::nav message){
		  has_heading = true;
  		this->current_heading = message;
	  }

		GateLocateProcedure()
		: n{},
			set_heading(n.serviceClient<navigation::nav_request>("/navigation/set_heading")),
      heading(n.subscribe("/navigation/heading", 1, &GateLocateProcedure::updateHeadingCallback, this)),
      full_stop_srv(n.serviceClient<navigation::full_stop>("/navigation/full_stop")),
      vision_vector(n.subscribe("/vision/GateDetector", 1, &GateLocateProcedure::vectorUpdateCallback, this)),
      has_gate_vector(false), has_heading(false), yaw_recorded(false), start_yaw(0.0),
      x(0), y(0), z(0)
		{ }

		GateLocateProcedure* clone() const override
		{
			return new GateLocateProcedure(*this);
		}

		Procedure::ReturnCode operator()() override {
  		// Ensure the procedure can access the data.
			if(!has_gate_vector) {
  		  ROS_WARN("GATE VECTOR NOT YET AVAILABLE DELAYING UNTIL AVAILABLE.");
				return Procedure::ReturnCode::CONTINUE;
			}

      if(!has_heading) {
        ROS_WARN("ORIENTATION NOT YET AVAILABLE DELAYING UNTIL AVAILABLE.");
        return Procedure::ReturnCode::CONTINUE;
      }

			// Set a new heading towards the gate

      //TO DO: Set yaw rate and rotate sub to same direction as vision-vector. Might have to adjust this on a sine function to keep the sub from going nuts.
      // and adjust depth based on z and the distance to the gate, unless that is ok as it is.
      // Once the yaw is set and the sub rotates (see rotation example below), Then STOP.
      // Then we will return NEXT (forward procedure to go forward through the gate).

			navigation::nav_request srv;
			srv.request.depth = current_heading.direction.z + z;
			srv.request.yaw_rate = 0.0;
			srv.request.forwards_velocity = 0.0;
			srv.request.sideways_velocity = 0.0;

			if (!set_heading.call(srv))
			{
				ROS_WARN("Heading service call failed.");
        return Procedure::ReturnCode::CONTINUE;
			}

      if(std::abs(current_heading.orientation.yaw - start_yaw) < 0.1) {
        navigation::full_stop stop;
        if(!full_stop_srv.call(stop)) {
          ROS_WARN("Full stop service call failed.");
        }
        return Procedure::ReturnCode::NEXT;
      }

      return Procedure::ReturnCode::CONTINUE;
		}
};

class DiveProcedure : public Procedure {
  	ros::NodeHandle n;
  	ros::ServiceClient set_heading;
  	ros::Subscriber depth;

  	double desired_depth;
  	double current_depth;
	public:

		void depthUpdateCallback(navigation::depth_info message)
		{
				desired_depth = message.desired_depth;
				current_depth = message.current_depth;
		}

		DiveProcedure()
		: n{},
		set_heading(n.serviceClient<navigation::nav_request>("/navigation/set_heading")),
		depth(n.subscribe("/navigation/depth", 1, &DiveProcedure::depthUpdateCallback, this))
		{}

		DiveProcedure* clone() const override
		{
			return new DiveProcedure(*this);
		}

		Procedure::ReturnCode operator()() override
		{
			navigation::nav_request srv;

			srv.request.depth = 0.5;
			srv.request.yaw_rate = 0;
			srv.request.forwards_velocity = 0;
			srv.request.sideways_velocity = 0;

			if (!set_heading.call(srv))
			{
				ROS_WARN("Heading service call failed.");
			}

			if(std::abs(desired_depth - current_depth) < 0.01)
				return Procedure::ReturnCode::NEXT;

		return Procedure::ReturnCode::CONTINUE;
		}
};

class IdleProcedure : public Procedure {
	  ros::NodeHandle n;
	  ros::ServiceClient set_heading;
	public:
		IdleProcedure()
		: n{}, set_heading(n.serviceClient<navigation::nav_request>("/navigation/set_heading"))
		{}
		IdleProcedure* clone() const override
		{
			return new IdleProcedure(*this);
		}

		Procedure::ReturnCode operator()() override {
			navigation::nav_request srv;


			srv.request.depth = 0;
			srv.request.yaw_rate = 0;
			srv.request.forwards_velocity = 0;
			srv.request.sideways_velocity = 0;

			if (!set_heading.call(srv))
			{
				ROS_WARN("Heading service call failed.");
			}

			return Procedure::ReturnCode::CONTINUE;
		}
};

class RotateProcedure : public Procedure {
		private:
			ros::NodeHandle n;
			ros::ServiceClient full_stop_srv;
			ros::Subscriber heading;

			bool yaw_recorded;
			double start_yaw;
			bool has_heading;
			navigation::nav current_heading;
		public:
			RotateProcedure(){

			}
			RotateProcedure* clone() const override{
				return new RotateProcedure(*this);
			}
			Procedure::ReturnCode operator()() override{

				return Procedure::ReturnCode::CONTINUE;
			}

};


class RotateRightAngleProcedure: public Procedure {
		ros::NodeHandle n;
		ros::ServiceClient set_heading;
		ros::ServiceClient full_stop_srv;
		ros::Subscriber heading;

		bool yaw_recorded;
		double start_yaw;
		bool has_heading;
		navigation::nav current_heading;

	public:
  	void updateHeadingCallback(navigation::nav message){
		  has_heading = true;
  		this->current_heading = message;
	  }

		RotateRightAngleProcedure()
		: n{},
			set_heading(n.serviceClient<navigation::nav_request>("/navigation/set_heading")),
			full_stop_srv(n.serviceClient<navigation::full_stop>("/navigation/full_stop")),
			heading(n.subscribe("/navigation/heading", 1, &RotateRightAngleProcedure::updateHeadingCallback, this)),
			//current_heading(nullptr),
			has_heading(false), yaw_recorded(false), start_yaw(0.0)
		{ }

		RotateRightAngleProcedure* clone() const override
		{
			return new RotateRightAngleProcedure(*this);
		}

		Procedure::ReturnCode operator()() override {
  		// Ensure the procedure can access the data.
			if(!has_heading) {
  		  ROS_WARN("ORIENTATION NOT YET AVAILABLE DELAYING UNTIL AVAILABLE.");
				return Procedure::ReturnCode::CONTINUE;
			}

			// After we have access to the data record the yaw.
			if(!yaw_recorded) {
				start_yaw = current_heading.orientation.yaw;
				yaw_recorded = true;
			}

			// Start rotation.
			navigation::nav_request srv;
			srv.request.depth = current_heading.direction.z;
			srv.request.yaw_rate = 5; // TODO TUNE PARAMETER IDK WHAT UNIT THIS IS IN
			srv.request.forwards_velocity = 0;
			srv.request.sideways_velocity = 0;

			if (!set_heading.call(srv))
			{
				ROS_WARN("Heading service call failed.");
			}

			if(current_heading.orientation.yaw - start_yaw > 90) {
				navigation::full_stop stop;
				if(!full_stop_srv.call(stop)) {
					ROS_WARN("Full stop service call failed.");
				}
				return Procedure::ReturnCode::NEXT;
			}

			return Procedure::ReturnCode::CONTINUE;
		}
};


class ForwardsProcedure : public Procedure {
  	ros::NodeHandle n;
  	ros::ServiceClient set_heading;
  	ros::ServiceClient full_stop;
  	ros::Subscriber heading;

		bool has_heading;
  	navigation::nav current_heading;
  	bool set_time;
  	ros::Time start_time;

	public:

		void updateHeadingCallback(navigation::nav message) {
			has_heading = true;
			current_heading = message;
		}

		ForwardsProcedure()
		: n{},
		set_heading(n.serviceClient<navigation::nav_request>("/navigation/set_heading")),
			full_stop(n.serviceClient<navigation::full_stop>("/navigation/full_stop")),
			heading(n.subscribe("/navigation/heading", 1, &ForwardsProcedure::updateHeadingCallback, this)),
			has_heading(false), set_time(false), start_time{}
		{ }

		ForwardsProcedure* clone() const override {
			return new ForwardsProcedure(*this);
		}

		Procedure::ReturnCode operator()() override {
				if(!has_heading) {
					ROS_WARN("ORIENTATION NOT YET AVAILABLE DELAYING UNTIL AVAILABLE.");
					return Procedure::ReturnCode::CONTINUE;
				}

			if(!set_time) {
					start_time = ros::Time::now();
					set_time = true;
				}

				navigation::nav_request srv;
				srv.request.depth = current_heading.direction.z;
				srv.request.forwards_velocity = 1;
				srv.request.sideways_velocity = 0;
				srv.request.yaw_rate = 0;

			if (!set_heading.call(srv)) {
				ROS_WARN("Heading service call failed.");
			}

			if((ros::Time::now() - start_time).sec >= 3) {
				return Procedure::ReturnCode::NEXT;
			}

			return Procedure::ReturnCode::CONTINUE;
		}

};

class SurfaceProcedure : public Procedure {
			ros::NodeHandle n;
			ros::ServiceClient set_heading;

		public:
			SurfaceProcedure()
			: n{}, set_heading(n.serviceClient<navigation::nav_request>("/navigation/set_heading"))
			{}
			SurfaceProcedure* clone() const override
			{
				return new SurfaceProcedure(*this);
			}

			Procedure::ReturnCode operator()() override {
				navigation::nav_request srv;

				srv.request.depth = 0;
				srv.request.yaw_rate = 0;
				srv.request.forwards_velocity = 0;
				srv.request.sideways_velocity = 0;

				if (!set_heading.call(srv))
				{
					ROS_WARN("Heading service call failed.");
				}

				return Procedure::ReturnCode::CONTINUE;
			}
};


class ProcedureA : public Procedure {
		std::size_t iterations;

	public:
		ProcedureA() = default;

		ProcedureA* clone() const override
		{
			return new ProcedureA(*this);
		}

		Procedure::ReturnCode operator()() override
		{
			if(iterations == 5)
				return Procedure::ReturnCode::NEXT;
			++iterations;
			ROS_INFO_STREAM("ProcedureA");
			return Procedure::ReturnCode::CONTINUE;
		}
};

class ProcedureB : public Procedure {
			std::size_t iterations;

		public:
			ProcedureB() = default;

			ProcedureB* clone() const override
			{
				return new ProcedureB(*this);
			}

			Procedure::ReturnCode operator()() override
			{
				if(iterations == 5)
					return Procedure::ReturnCode::NEXT;
				++iterations;
				ROS_INFO_STREAM("ProcedureB");
				return Procedure::ReturnCode::CONTINUE;
			}
};

}

#endif
