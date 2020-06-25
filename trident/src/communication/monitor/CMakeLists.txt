cmake_minimum_required(VERSION 2.8.3)
project(monitor)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(Boost_USE_MULTITHREADED OFF)
#####################################################################################
# Catkin                                                                            #
#####################################################################################

## What packages to we need to compile with our package ?
find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  message_generation
  can_msgs
)

find_package(Boost REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   GetADevice.srv
   GetAllDevices.srv
   GetCanMSG.srv
)

#add_message_files(
 # FILES
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   can_msgs
)

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


#####################################################################################
# Binaries                                                                          #
#####################################################################################


# can_manager_node
add_executable(can_manager_node src/can_manager_node.cpp include/can_manager.hpp)
add_dependencies(can_manager_node monitor_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_include_directories(can_manager_node PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(can_manager_node ${catkin_LIBRARIES})

# powerboard_node
add_executable(powerboard_node src/powerboard_node.cpp include/peripheral_manager.hpp)
add_dependencies(powerboard_node monitor_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(powerboard_node ${catkin_LIBRARIES})

# jetson_diagnostics_node
add_executable(jetson_diagnostics_node src/jetson_diagnostics_node.cpp)
target_link_libraries(jetson_diagnostics_node ${catkin_LIBRARIES})
add_dependencies(jetson_diagnostics_node monitor_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
