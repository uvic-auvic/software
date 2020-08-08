cmake_minimum_required(VERSION 3.0.2)
project(auvic_msgs)

find_package(catkin REQUIRED
  can_msgs 
  message_generation 
  std_msgs 
  geometry_msgs
)
add_service_files(
  FILES
  Monitor_Send_Data_To_World.srv
  Devices_Get_Data_From_Peripherals.srv
)

add_message_files(
  FILES
  Monitor_Jetson_To_World.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    can_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)

include_directories(
   ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)