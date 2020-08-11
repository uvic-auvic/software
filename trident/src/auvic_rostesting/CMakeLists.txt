cmake_minimum_required(VERSION 3.0.2)
project(auvic_rostesting)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED COMPONENTS
    devices
    monitor
    auvic_msgs
    can_msgs
    roscpp
    rospy
    rostest 
)

catkin_package(
   CATKIN_DEPENDS
    roscpp
    rospy
    auvic_msgs
    can_msgs
)

catkin_install_python(PROGRAMS scripts/powerboardSimulator.py scripts/ExampleApplicationNode.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
   ${catkin_INCLUDE_DIRS}
)

# example_app_client
#add_executable(example_app_client src/example_app_client)
#target_link_libraries(example_app_client ${catkin_LIBRARIES})

install(
  TARGETS
    #example_app_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)