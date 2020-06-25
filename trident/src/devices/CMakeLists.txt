cmake_minimum_required(VERSION 3.0.2)
project(devices)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  can_msgs
  roscpp
  message_generation
  std_msgs
)

# Generate services in the 'srv' folder
#add_service_files(
#   FILES
#)

#add_message_files(
 # FILES
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   std_msgs
#   can_msgs
#)

catkin_package(
   INCLUDE_DIRS
      include
   CATKIN_DEPENDS
      can_msgs
      message_runtime
      std_msgs
      roscpp
)

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)

# Declare a C++ executable
#add_executable(dropper_node src/dropper_node.cpp include/peripheral_manager.hpp)
#add_executable(dvl_node src/dvl_node.cpp include/peripheral_manager.hpp)
#add_executable(grabber_node src/grabber_node.cpp include/peripheral_manager.hpp)
#add_executable(hydrophone_node src/hydrophone_node.cpp include/peripheral_manager.hpp)
#add_executable(imu_node src/imu_node.cpp include/peripheral_manager.hpp)
#add_executable(lcd_board_node src/lcd_board_node.cpp include/peripheral_manager.hpp)
#add_executable(motorcontroller_node src/motorcontroller_node.cpp include/peripheral_manager.hpp)



# powerboard_node
add_executable(powerboard_node src/powerboard_node.cpp include/peripheral_manager.hpp)
add_dependencies(powerboard_node monitor_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(powerboard_node ${catkin_LIBRARIES})

#add_executable(torpedo_node src/torpedo_node.cpp include/peripheral_manager.hpp)
