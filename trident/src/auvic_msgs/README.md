# auvic_msgs

This package contains the message and service files needed to publish to topics and connect with nodes, respectively. I want us to stop creating messages/services within the main packages because this caused us dependency errors on ROS Kinetic; The only work around I found is to have a dedicated package for msgs/srvs.

Note: ros does not support sub folders in the srv and msg folders, therefore, we will be following the naming structure: **packageName_nameOfMessage** (ex. Monitor_Jetson_To_World) 

(IDK if you use camelCase or Underline, but package name is a must. This keeps the same messages/services together.)

# Messages vs Services vs Actions
- Messages: Used for continuous data streams. No feedback. Used by topics.
    - IE: Camera feed, depth data, Orientation, etc.
- Services: Simple blocking call, used for *fast* tasks requesting specific data. Has feedback.
    - IE: Asking for calcualtion, asking for state of node, requesting information, etc.
- Actions: More complex non-blocking background processing. They can be pre-empted but should only be used via action servers. Used for longer tasks. Has feedback and timeout condition.
    - IE: Running thrusters, gathering camera footage, etc.

Services can be used as messages but not vice-versa; the response section of a service can be kept empty to avoid the overhead of feedback.

# How to make a message
1. Create a message file in msg folder:
    - "Devices_ThisIsARandomMessage.msg"
    - Inside, add some message types (uint8, std_msgs/String, uint64[8], etc)
2. Add it to the CMakeLists.txt file:
    - add_message_files( FILES Monitor_Jetson_To_World.msg Devices_ThisIsARandomMessage.msg)
3. Done. Now just call it in other packages by:
    - #include <auvic_msgs/Devices_ThisIsARandomMessage.h>
    - auvic_msgs::Devices_ThisIsARandomMessage object;

# How to make a service
1. Create a service file in the srv folder:
    - "Devices_ThisIsARandomService.srv"
    - Services have two parts: request and response. they are seperated via "---" symbols (3 hyphens).
2. Add it to the CMakeLists.txt file:
    - add_Service_files( FILES Monitor_Send_Data_To_World.srv Devices_ThisIsARandomService.srv)
3. Done. Now just call it in other packages by:
    - #include <auvic_msgs/Devices_ThisIsARandomService.h>
    - auvic_msgs::Devices_ThisIsARandomService object;
4. Extra: If you added a response field, the server may or may not populate it, but all further actions in the client node will be suspended until the server is finished.

# How to make a action
- Will add in the future if needed. Probably not. well... depends on 1AI.