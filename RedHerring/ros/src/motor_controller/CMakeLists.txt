#####################################################################################
# CMake                                                                             #
#####################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(motor_controller)
set(CMAKE_CXX_FLAGS "-std=c++11")

#####################################################################################
# Catkin                                                                            #
#####################################################################################

## What packages to we need to compile with our package ?
find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  message_generation
  serial
  system_manager
)
## Generate services in the 'srv' folder
add_service_files(
   FILES
   motor_command.srv
   movement_command.srv
)

add_message_files(
  FILES
  keyboard.msg
  position.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
${catkin_INCLUDE_DIRS})

#####################################################################################
# Binaries                                                                          #
#####################################################################################

## Required packages for linking
find_package(serial REQUIRED)

## Declare a C++ executable
add_executable(controller src/controller.cpp)
add_dependencies(controller ${motor_controller_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${system_manager_EXPORTED_TARGETS})
target_link_libraries(controller ${catkin_LIBRARIES} ${serial_LIBRARIES})

add_executable(movement src/movement.cpp)
add_dependencies(movement ${motor_controller_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(movement ${catkin_LIBRARIES} ${serial_LIBRARIES})

add_executable(client src/client.cpp)
add_dependencies(client ${motor_controller_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(client ${catkin_LIBRARIES} ${serial_LIBRARIES})
