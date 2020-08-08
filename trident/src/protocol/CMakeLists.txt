cmake_minimum_required(VERSION 3.0.2)
project(protocol)

catkin_package( INCLUDE_DIRS include)

include_directories( include ${catkin_INCLUDE_DIRS})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)