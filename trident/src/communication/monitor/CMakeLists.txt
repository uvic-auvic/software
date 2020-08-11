cmake_minimum_required(VERSION 3.0.2)
project(monitor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED
  roscpp
  auvic_msgs
  can_msgs
  protocol
  rosunit
)

catkin_package(
   INCLUDE_DIRS
      include
   LIBRARIES
      HardwareToSoftware
      SoftwareToHardware
   CATKIN_DEPENDS
      roscpp
      auvic_msgs
      can_msgs
)

include_directories(
   include
   ${auvic_msgs_INCLUDE_DIRS}
   ${catkin_INCLUDE_DIRS}
   ${protocol_INCLUDE_DIRS}
   
)

# SoftwareToHardware
add_library(SoftwareToHardware src/SoftwareToHardware.cpp)
add_dependencies(SoftwareToHardware ${catkin_EXPORTED_TARGETS})
target_link_libraries(SoftwareToHardware ${catkin_LIBRARIES})

# SoftwareToHardware_node
add_executable(SoftwareToHardware_node src/SoftwareToHardware_node.cpp include/can_manager.hpp)
add_dependencies(SoftwareToHardware_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(SoftwareToHardware_node SoftwareToHardware ${catkin_LIBRARIES})

# HardwareToSoftware
add_library(HardwareToSoftware src/HardwareToSoftware.cpp)
add_dependencies(HardwareToSoftware ${catkin_EXPORTED_TARGETS})
target_link_libraries(HardwareToSoftware ${catkin_LIBRARIES})

# HardwareToSoftware_node
add_executable(HardwareToSoftware_node src/HardwareToSoftware_node.cpp include/can_manager.hpp)
add_dependencies(HardwareToSoftware_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(HardwareToSoftware_node HardwareToSoftware ${catkin_LIBRARIES})

# this adds these nodes to the build folder. for c++
install(
  TARGETS
    #can_manager
    #can_manager_node
    HardwareToSoftware
    HardwareToSoftware_node
    SoftwareToHardware
    SoftwareToHardware_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  #this line adds python test
  #add_rostest(test/test_node_HS.test)

  #this line adds c++ test
  add_rostest_gtest(test_node_SH test/test_node_SH.test test/test_node_SH.cpp)
  target_link_libraries(test_node_SH ${catkin_LIBRARIES})
  add_dependencies(test_node_SH ${catkin_EXPORTED_TARGETS})

endif()

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)