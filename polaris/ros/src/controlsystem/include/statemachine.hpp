#ifndef STATEMACHINE_HPP
#define STATEMACHINE_HPP

#include <ros/ros.h>
#include <ros/console.h>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <string>
#include <stack>
#include <stdexcept>

#include "procedures.hpp"


class StateMachine {
public:
  // Converting control terminology to state machine terminology.
  using symbol = procedures::Procedure::ReturnCode;
  using state_index = std::size_t;

  // Member for verbosely describing states.
  struct State {
    std::unique_ptr<procedures::Procedure> procedure;
    std::string error;
    std::string next;
    bool has_params;
    bool params; // TODO add support for this.

    std::string path;
    std::string name;

    State(State&&) = default;
    State& operator=(State&&) = default;
  };

  enum struct TransitionIndex : int {
  	NEXT = 0,
  	ERROR = 1,
  	FATAL = 2,
  };

  enum struct StepResult : int {
  	CONTINUE = 0,
  	EXITSUCCESS = 1,
  	EXITFAILURE = 2,
  };

  static std::string to_string(StepResult x)
  {
    switch(x)
    {
    	case StepResult::CONTINUE:
    		return "CONTINUE";
    	case StepResult::EXITSUCCESS:
    	  return "EXITSUCCESS";
    	case StepResult::EXITFAILURE:
    		return "EXITFAILURE";
    	default:
    		return "UNDEFINED";
    }

  }

private:
	using functormap_type = std::map<std::string, std::unique_ptr<procedures::Procedure>>;

  // List of states that the state machine consists of.
  std::vector<State> states;
  state_index current;
  state_index start;
  // Transition table, rows are for transitions as follows
  // 0: next, 1: error, 2: fatal
  /*
    This is advantageuous because transitions can be performed in constant
    time, and the size_t maps directly back to states, which means states
    can be quickly. This results in the entire transition process being as
    efficient as it can be.
   */
  std::array<std::array<state_index, 64ul>, 3ul> transitions;

  static void load_functors_(functormap_type& functor_map);

  bool _check_member(XmlRpc::XmlRpcValue& o, const char* m);

  static std::string _features_to_string(std::size_t features);

  // Used to read states from passed XmlRpcValue structure.
  void _read_states(XmlRpc::XmlRpcValue& state_list);
  // Generates transition functions in the state machine.
  void _gen_machine();

public:
  // Constructs a empty state machine.
  StateMachine();
  // Constructs a state machine from a state description.
  explicit StateMachine(XmlRpc::XmlRpcValue& state_list);

  StepResult operator()();

  void print_debug();
};


#endif
