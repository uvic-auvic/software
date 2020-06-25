cmake_minimum_required(VERSION 3.0.2)
project(peripherals)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  can_msgs
  roscpp
  message_generation
  std_msgs
  monitor
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


add_executable(powerboard_node src/powerboard_node.cpp include/peripheral_manager.hpp)
target_link_libraries(powerboard_node ${catkin_LIBRARIES})


#add_executable(torpedo_node src/torpedo_node.cpp include/peripheral_manager.hpp)