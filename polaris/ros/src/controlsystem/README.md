# StateMachine configuration through YAML.

## General Understanding Behind Development

The general form of the YAML configuration file for the control systems state machine is as follows.

The root tag containing the description of the state machines states is labeled 'states'. Nested within
the root tag is a list of state lists, or actual states themselves. The reason state lists are offered
is to provide an identification system behind the tasks.

An example of why this is useful is the gate stage. Just for managing the Gate portion of the competition
there are several different stages the submarine needs to go through. Having lists of task lists
clustered under the same namespace makes understanding how the tasks are related easier. This allows the
seperation of different actions under the same general task. A YAML example of this can be seen below.

```yaml
states:

  # ...

  somestate:
    procedure: SomeProc
    error: surface
    next: buoy/locate
  buoy:
    locate:
      procedure: BuoyDetectProc
      error: surface # probably don't want to just give up
      next: buoy/orient
    orient:
      procedure: BuoyOrientProc
      params:
        side: vampire
        timeout: 45
      error: buoy/locate
      next: surface # probably want to actually proceed 

  # ...
```

## Implimentation Spec

The YAML file MUST have a root tag 'states' which is a dictionary of either all state OR state lists. To differentiate
between a state and a state list restrictions must be placed on what MUST be in a state, and what
MUST NOT be in a state list. It is also to note that the root dictionary states, would be classified
as a state list.

State lists are dictionaries containing keys being the state names, and values being the state parameters.
State lists MUST NOT use reserved words (listed below under 'State Reserved Words') or required state names
(listed below under 'Required States') as their names.

States are dictionaries. States MUST have the following dictionary item fields: 'procedure' which
names some defined C++ function in header/procedures.hpp, error which names some defined path to a
state in this YAML file, and next which names some defined path to a state in this YAML file.
What is meant by 'path to state' is if a state is in the root state list it is simply labeled as it
is named, however if a state is within a state list(s) within the root state list it's label is a
list of states lists from outermost to innermost seperated by '/', with the state name separated from
the last state list by a '/' (see example below). States CAN have the following fields, but are NOT
REQUIRED to: params. States MUST NOT use reserved words (listed below) as their names.

### State Reserved Words
- "procedure": This defines the c++ procedure that this state relies upon.
- "error": This specifies the path/name of the state this state transitions to for error behaviour.
- "next": This specifies the path/name of the state this state transitions to for normal behaviour.
- "params": This specifies a dictionary of options for the state to be created with. (This needs to be defined.)

### Required States
These states cannot be part of a state list subordinate to the root state list, as this would alter their global name.
This list is subject to change as newer and better ideas for naming and state orderings rear their ugly heads.
- "dive": This currently represents the start state for the state machine.
- "surface": This currently represents the end state for the state machine. (This will more than likely need to change.)

```yaml
states:
  startstate:
    # ...
    # my label is 'startstate'
  aaaa:
    xd:
      # ...
      # label is 'aaaa/xd'
   woah:
      finallyastate:
        # ...
        # my label is 'states/aaaa/woah/finallyastate'
```

## Possible Future Additions

An idea for possible future adaptations of the control system would be to allow the programmer and configuration
writer with the ability to specify multiple _next_ and _error_ states, in the form of a list. This could allow
for more specified behaviour from what actually results in the state switching. Which leads to less complicated
procedure routines, as they can make more specific assumptions. However, this leads to the possible need for more
procedures to be added.

# Procedures

## Creating a Procedure

```c++
class DerivedProcedure : public Procedure {
    DerivedProcedure()
    {
        // Possible default configuration.
    }

    // This function should be implimented this way or
    // weird things will happen.
    DerivedProcedure* clone() const override
    {
        return new DerivedProcedure(*this);
    }

    /*!
     * THIS IS NOT IMPLIMENTED
     * Provide a description of what the params are and do
     * @params:
     *  - param_name: description
     */
    virtual DerivedProcedure& operator=(XmlRpcValue& params)
    {
    
    }

    /*!
     * Provide a meaningful description of what the derived procedure does.
     * @dependencies: list all the information this functor needs to work.
     */
    Procedure::ReturnCode operator()() override
    {
        // Measure something

        // Do something from what was measured.

        // Tell the program what to do next.
        return Procedure::ReturnCode::CONTINUE;
    }
};
```

## Interfacing with the AUV

This is an example of using a service in a procedure. This example uses the service at `/vision/change_detection`.
With service type of `vision::change_detection`.

```c++
class NewProcedure : public Procedure {
    ros::NodeHandle nh;
    
    ros::ServiceClient change_detection;

public:  
    NewProcedure()
    :change_detection(nh.serviceClient<vision::change_detection>("/vision/change_detection")
    {} // Unused body.

    // Clone procedure.

    Procedure::ReturnCode operator()() override {
        // Changing Detection Type (might not want to call this every time)
        vision::change_detection srv;
        srv.message.enabled_type = EnabledDetector::BUOY;
    
        if(change_detection.call(srv)) {
            // SUCCESS
        } else {
            // FAIL
        }
        return Procedure::ReturnCode::CONTINUE;
    }
}
```

This is an example of using a topic in a procedure. This example uses the publisher at `/vision/vector`.
With a message type of `vision::vector`.

```c++
class NewProcedure : public Procedure {
    ros::NodeHandle nh;
    
    ros::Subscriber vision_vector;
    
    uint16_t x, y, z;

public: 
    void vectorUpdateCallback(const vision::vector::ConstPtr& message)
    {
        x = message.x;
        y = message.y;
        z = message.z;
    }
    
    // Clone procedure.
    
    NewProcedure()
    :vision_vector(nh.subscribe("/vision/vector", 1, &NewProcedure::vectorUpdateCallback, this))
    {} // Unused body.

    Procedure::ReturnCode operator()() override {
        // The vector information stored in x/y/z will update whenever the vision node updates.
        // So no checking needs to be done.
        // We can simply just access the data and make the assumption it is the most recent, because it will be.

        return Procedure::ReturnCode::CONTINUE;
    }
}
```

### Computer Vision
The vector result from detection can be found with the `/vision/vector` topic. And what the detection system is looking for can be changed through the service `/vision/change_direction`. It is to note that the service takes in a 8-bit unsigned integer, it is recommended to use the enum class in the vision package's include `EnabledDetector.hpp`

Results of detection can be obtained through a `ros::Subscriber` at `/vision/vector` these messages are defined as follows.
```
uint16 x
uint16 y
uint16 z
```

To request a different type of detection the use of a `ros::ServiceClient` at `/vision/change_detection` is required.
This service (`vision::change_detection`) is defined as follows. Note request is top and response is the bottom. Note the response is empty.
To ensure you're requesting the right type of detection you must refer to the `EnabledDetector` enum class in `vision/include/EnabledDetector.hpp`
```
uint8 enabled_type
---
```


### Movement Related Functions

To change the heading of the submarine use the `ros::ServiceClient` at `/navigation/set_heading` of type `navigation::nav_request`.
This only requires the following fields to be filled out for the request.

```
float64 depth
float64 yaw_rate
float64 forwards_velocity
float64 sideways_velocity
---
```


To request the submarine to perform a full stop use the `ros::SericeClient` at `/navigation/full_stop` of type `navigation::full_stop`
This service simply needs to be called to hault the movement of the AUV.
```

---
```

To get information about the current heading utilize the following topics.

```
ros::Subscriber @ /navigation/heading
ros::Subscriber @ /navigation/depth
```
