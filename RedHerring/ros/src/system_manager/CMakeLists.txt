#####################################################################################
# CMake                                                                             #
#####################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(system_manager)
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
)
## Generate services in the 'srv' folder
add_service_files(
   FILES
   device_manager.srv
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
add_executable(device_manager src/device_manager.cpp)
add_dependencies(device_manager ${system_manager_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(device_manager ${catkin_LIBRARIES} ${serial_LIBRARIES})

