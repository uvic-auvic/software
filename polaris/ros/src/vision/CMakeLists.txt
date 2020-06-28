##############################################################################
# CMake                                                                      #
##############################################################################
cmake_minimum_required(VERSION 2.8.3)
project(vision)
set(CMAKE_CXX_FLAGS "-std=c++11")

##############################################################################
# Catkin                                                                     #
##############################################################################

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

add_service_files(
  FILES
  change_detection.srv
)

add_message_files(
  FILES
  vector.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# This is useful when we want other packages to be able to use this package
# E.g. if another package wanted to include a few headers from here
# To run in their own nodes, they would include this package "vision"
# And we would uncomment INCLUDE_DIRS, but generally everything happens here
# SO theres no need (for now)
catkin_package(
#  INCLUDE_DIRS include
  #LIBRARIES vision
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs message_runtime std_msgs message_generation
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

##############################################################################
# Binaries                                                                   #
##############################################################################

# Required packages
find_package(OpenCV 3 REQUIRED)

# Separate nodes in the package
add_executable(${PROJECT_NAME}_node src/vision_node.cpp src/CameraInput.cpp src/BuoyDetector.cpp src/GateDetector.cpp src/PathDetector.cpp src/Distance.cpp src/Filter.cpp)
add_dependencies(${PROJECT_NAME}_node ${vision_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
