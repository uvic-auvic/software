#include "statemachine.hpp"

#include <memory>

// This macro is used to allocate memory for each functor that's managed by a unique smart pointer.
// These objects are copied later.
#define FUNC_INSERT(x,y) functor_map.insert(std::make_pair<std::string, std::unique_ptr<procedures::Procedure>>((x), std::unique_ptr<procedures::Procedure>((new y))))
void StateMachine::load_functors_(StateMachine::functormap_type& functor_map)
{
	FUNC_INSERT("DiveProcedure", procedures::DiveProcedure());
	FUNC_INSERT("ProcedureA", procedures::ProcedureA());
	FUNC_INSERT("ProcedureB", procedures::ProcedureB());
	FUNC_INSERT("IdleProcedure", procedures::IdleProcedure());
	FUNC_INSERT("SurfaceProcedure", procedures::SurfaceProcedure());
  FUNC_INSERT("RotateRightAngleProcedure", procedures::RotateRightAngleProcedure());
  FUNC_INSERT("RotateProcedure", procedures::RotateProcedure());
}

  // Private member function used to check if a parameter.
bool StateMachine::_check_member(XmlRpc::XmlRpcValue& o, const char * m){
  if(o.hasMember(m)) {
    switch(o[m].getType()) {
    case XmlRpc::XmlRpcValue::TypeString:
      return true;
    case XmlRpc::XmlRpcValue::TypeStruct:
      if(!std::strcmp(m, "params"))
        return true;
    default:
      ROS_FATAL_STREAM("Parameter parse error. " << m);
    }
  }
  return false;
}

// Error message function, used for converting found features
// to a human-readable message.
std::string StateMachine::_features_to_string(const std::size_t features)
{
  std::string result;
  if (features & 0b1000u) {
	  result += std::string("procedure");
	  if (!(features & 0b0111u))
	  	result += ", ";
  }
  if (features & 0b0100u) {
	  result += std::string("error");
	  if (!(features & 0b1011u))
	  	result += ", ";
  }
  if (features & 0b0010u) {
	  result += "next";
	  if (!(features & 0b1101u))
	  	result += ", ";
  }
  if (features & 0b0001u)
    result += "params";
  return result;
}

void StateMachine::_read_states(XmlRpc::XmlRpcValue& state_list)
{
  // Statically load functors from procedures.hpp
  StateMachine::functormap_type functor_map;
  StateMachine::load_functors_(functor_map);

  // Assert proper types.
  ROS_ASSERT(state_list.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  states.reserve(64);

  // State list namespaces & processing stack.
  std::stack<std::pair<std::string,XmlRpc::XmlRpcValue>> state_list_stack;
  state_list_stack.push(std::make_pair("", state_list));

  // For each state_list, iterate over all items in that state list.
  // If the item is a state, add it to states.
  // If the item is a state_list, add it to the state_list_stack of things to process.
  while(!state_list_stack.empty()) {
    std::string path = state_list_stack.top().first;
    XmlRpc::XmlRpcValue current = state_list_stack.top().second;
    ROS_INFO_STREAM(path);
    state_list_stack.pop();

    for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = current.begin(); it != current.end(); ++it)
      {
        // Create a usable copy.
        XmlRpc::XmlRpcValue second = it->second;

        ROS_ASSERT(second.getType() == XmlRpc::XmlRpcValue::TypeStruct);

        // Determine if the element is a state.
        std::size_t features = 0b0000;
        if(_check_member(second, "procedure")) features |= 0b1000u;
        if(_check_member(second, "error")) features |= 0b0100u;
        if(_check_member(second, "next")) features |= 0b0010u;
        if(_check_member(second, "params")) features |= 0b0001u;

        // Was a valid state found.
        StateMachine::State new_state = {};
        switch(features) {
        case 0b1111: // Found state with all features, with parameters.
          // TODO add parameter support.
          new_state.has_params = true;
          new_state.params = false;
        case 0b1110: // Found state with all features, without parameters.
          // State without parameters
          new_state.name = static_cast<std::string>(it->first);
          new_state.path = path;
          try {
            // Look-up function definition.
//            new_state.procedure = std::unique_ptr<procedures::Procedure>(*functor_map.at(static_cast<std::string>(second["procedure"]))));
						new_state.procedure = std::unique_ptr<procedures::Procedure>((*functor_map.at(static_cast<std::string>(second["procedure"]))).clone());
          } catch (const std::out_of_range& oor) {
            ROS_FATAL_STREAM("Invalid configuration or source. State "
                             << new_state.path << new_state.name
                             << " references functor "
                             << static_cast<std::string>(second["procedure"])
                             << " which cannot be found.");
            ROS_FATAL_STREAM("Check spelling, if it's loaded, or if it still needs to be implimented.");
            throw std::invalid_argument("Referencing non-existent functor.");
          }
          new_state.error = static_cast<std::string>(second["error"]);
          new_state.next = static_cast<std::string>(second["next"]);

          // Log state was found.
          ROS_INFO_STREAM("State [" << new_state.path << new_state.name << "]");
          states.push_back(std::move(new_state));
          break;
        case 0b0000:
          ROS_DEBUG_STREAM("State List " << static_cast<std::string>(it->first));
          state_list_stack.push(std::make_pair(path + it->first + '/', it->second));
          break;
        default:
          ROS_FATAL_STREAM("Invalid configuration provided. Found token(s) "
                           << StateMachine::_features_to_string(features)
                           << " in state which is not allowed.");
          throw std::invalid_argument("State machine configuration YAML.");
        }
      }
  }
}

void StateMachine::_gen_machine()
{
  bool found_start = false;
  bool found_end = false;
  state_index end = 65;
  for(state_index i = 0; i < states.size(); ++i)
    {
      // Using as a macro
      const State& s = states[i];
      // Need to find the start state, "dive"
      if(s.name == "dive") {
        if (found_start)
          throw std::invalid_argument("State machine configuration YAML. Multiple start (dive) states found.");
        start = i;
        found_start = true;
      }

      ROS_INFO_STREAM(s.path << s.name);

      // And to ensure there's an end.
      if (s.name == "surface") {
        if (found_end)
          throw std::invalid_argument("State machine configuration YAML. Multiple end (surface) states found.");
        end = i;
        found_end = true;
      }

      // Also while building the transition table.
      bool found_next = false;
      bool found_error = false;
      for(state_index j = 0; j < states.size(); ++j)
        {
          // Using as a macro
          const State& t = states[j];

          if(s.next == (t.path + t.name)) {
            if (found_next)
              throw std::invalid_argument("State machine configuration YAML. Multiple next transitions found.");
            found_next = true;
            transitions[0][i] = j;
          }
          if(s.error == (t.path + t.name)) {
            if (found_error)
              throw std::invalid_argument("State machine configuration YAML. Multiple error transitions found.");
            found_error = true;
            transitions[1][i] = j;
          }
        }
    }
  if(!found_start)
    throw std::invalid_argument("In state machine configuration YAML, start (dive) state not defined.");
  if(!found_end)
    throw std::invalid_argument("In state machine configuration YAML, end (surface) state not defined.");

  // Set all transitions for fatal error to end state.
  for(state_index i = 0; i < 64; ++i)
    transitions[2][i] = end;
}

StateMachine::StateMachine()
: current(0), start(0), transitions{} {}

// Need to pass in list of State structs,
// use that to track next state to go.
StateMachine::StateMachine(XmlRpc::XmlRpcValue& state_list)
: current(0), start(0), transitions{}
{
  _read_states(state_list);
  _gen_machine();
  this->current = this->start;
}

// Added to fix CLion error message.
#define INDEX_CAST(x) static_cast<std::array<StateMachine::state_index, 64>::size_type>(x)

StateMachine::StepResult StateMachine::operator()()
{
	ROS_INFO_STREAM("Current state" << this->states[this->current].name);
	// Use the result with the transition function.
	switch((*this->states[this->current].procedure)())
	{
		case procedures::Procedure::ReturnCode::CONTINUE:
			// Do nothing state.
			break;
		case procedures::Procedure::ReturnCode::NEXT:
			this->current = this->transitions[INDEX_CAST(StateMachine::TransitionIndex::NEXT)][this->current];
			break;
		case procedures::Procedure::ReturnCode::ERROR:
			this->current = this->transitions[INDEX_CAST(StateMachine::TransitionIndex::ERROR)][this->current];
			break;
		default:
			ROS_FATAL_STREAM("Cannot identify state to transition to." << this->states[this->current].name);
		case procedures::Procedure::ReturnCode::FATAL:
			this->current = this->transitions[INDEX_CAST(StateMachine::TransitionIndex::FATAL)][this->current];
			ROS_INFO_STREAM("failure");
			return StateMachine::StepResult::EXITFAILURE;
	}
	return StateMachine::StepResult::CONTINUE;
}

void StateMachine::print_debug()
{
	{
		for(std::size_t i = 0; i != states.size(); ++i)
		{
			ROS_INFO_STREAM("State [" << i << "]: " << states[i].name << "< next: " << transitions[0][i] << " error: " << transitions[1][i] << " >");
		}
	}
}