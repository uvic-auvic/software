cmake_minimum_required(VERSION 2.8.3)
project(rov_controls)
set(CMAKE_CXX_FLAGS "-std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_runtime
  motor_controller
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES red_joystick
  CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(joystick_node src/joystick_node.cpp src/joystick.cpp)
target_link_libraries(joystick_node ${catkin_LIBRARIES})

add_executable(keyboard_node src/keyboard_node.cpp)
target_link_libraries(keyboard_node ${catkin_LIBRARIES})
add_dependencies(keyboard_node ${vision_EXPORTED_TARGETS} ${motor_controller_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

