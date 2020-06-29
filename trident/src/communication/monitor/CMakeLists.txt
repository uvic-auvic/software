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
)

catkin_package(
   INCLUDE_DIRS
      include
   LIBRARIES
      can_manager
   CATKIN_DEPENDS
      roscpp
      auvic_msgs
      can_msgs
)

include_directories(
   include
   ${auvic_msgs_INCLUDE_DIRS}
   ${catkin_INCLUDE_DIRS}
)


# can_manager
add_library(can_manager src/can_manager.cpp)
add_dependencies(can_manager ${catkin_EXPORTED_TARGETS})
target_link_libraries(can_manager ${catkin_LIBRARIES})

# can_manager_node
add_executable(can_manager_node src/can_manager_node.cpp include/can_manager.hpp)
add_dependencies(can_manager_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(can_manager_node can_manager ${catkin_LIBRARIES})

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