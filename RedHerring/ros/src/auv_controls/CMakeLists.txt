cmake_minimum_required(VERSION 2.8.3)
project(auv_controls)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  message_runtime
  motor_controller
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
   CATKIN_DEPENDS std_msgs roscpp message_runtime
)

## Declare a C++ executable
add_executable(ai src/ai.cpp)
add_dependencies(ai ${motor_controller_EXPORTED_TARGETS} ${vision_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ai ${catkin_LIBRARIES})
