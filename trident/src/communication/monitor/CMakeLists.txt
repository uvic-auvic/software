cmake_minimum_required(VERSION 3.0.2)
project(monitor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED
  roscpp
  can_msgs
)

find_package(Boost REQUIRED)

add_service_files(
   FILES
   GetADevice.srv
   GetAllDevices.srv
   GetCanMSG.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   can_msgs
)

catkin_package(
   INCLUDE_DIRS
      include
   LIBRARIES
      can_manager
   CATKIN_DEPENDS
      can_msgs
      std_msgs
      roscpp
)

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)

# can_manager
add_library(can_manager
   src/can_manager.cpp
)
target_link_libraries(can_manager
   ${catkin_LIBRARIES}
)
add_dependencies(can_manager
   ${catkin_EXPORTED_TARGETS})


# can_manager_node
add_executable(can_manager_node src/can_manager_node.cpp include/can_manager.hpp)
target_include_directories(can_manager_node PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(can_manager_node can_manager ${catkin_LIBRARIES})

# jetson_diagnostics_node
#add_executable(jetson_diagnostics_node src/jetson_diagnostics_node.cpp)
#add_dependencies(jetson_diagnostics_node monitor_generate_messages_cpp ${catkin_EXPORTED_TARGETS})


install(
  TARGETS
    can_manager
    can_manager_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)