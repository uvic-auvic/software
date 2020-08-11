cmake_minimum_required(VERSION 3.0.2)
project(devices)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  auvic_msgs
  can_msgs
  rostest
)

catkin_package(
   INCLUDE_DIRS
      include
   LIBRARIES
      dropper
      dvl
      grabber
      hydrophone
      imu
      lcd_board
      motorcontroller
      powerboard
      torpedo
      sailorman
   CATKIN_DEPENDS
      roscpp
      auvic_msgs
      can_msgs
)

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)

# dropper
add_library(dropper src/dropper.cpp )
target_link_libraries(dropper ${catkin_LIBRARIES})
add_dependencies(dropper ${catkin_EXPORTED_TARGETS})

# dropper_node
add_executable(dropper_node src/dropper_node.cpp include/peripheral_manager.hpp)
target_link_libraries(dropper_node dropper    ${catkin_LIBRARIES})

# dvl
add_library(dvl src/dvl.cpp )
target_link_libraries(dvl ${catkin_LIBRARIES})
add_dependencies(dvl   ${catkin_EXPORTED_TARGETS})

# dvl_node
add_executable(dvl_node src/dvl_node.cpp include/peripheral_manager.hpp)
target_link_libraries(dvl_node dvl    ${catkin_LIBRARIES})

# grabber
add_library(grabber src/grabber.cpp )
target_link_libraries(grabber ${catkin_LIBRARIES})
add_dependencies(grabber   ${catkin_EXPORTED_TARGETS})

# grabber_node
add_executable(grabber_node src/grabber_node.cpp include/peripheral_manager.hpp)
target_link_libraries(grabber_node grabber    ${catkin_LIBRARIES})

# hydrophone
add_library(hydrophone src/hydrophone.cpp )
target_link_libraries(hydrophone ${catkin_LIBRARIES})
add_dependencies(hydrophone   ${catkin_EXPORTED_TARGETS})

# hydrophone_node
add_executable(hydrophone_node src/hydrophone_node.cpp include/peripheral_manager.hpp)
target_link_libraries(hydrophone_node hydrophone    ${catkin_LIBRARIES})

# imu
add_library(imu src/imu.cpp ) 
target_link_libraries(imu ${catkin_LIBRARIES})
add_dependencies(imu   ${catkin_EXPORTED_TARGETS})

# imu_node
add_executable(imu_node src/imu_node.cpp include/peripheral_manager.hpp)
target_link_libraries(imu_node imu    ${catkin_LIBRARIES})

# lcd_board
add_library(lcd_board src/lcd_board.cpp )
target_link_libraries(lcd_board ${catkin_LIBRARIES})
add_dependencies(lcd_board   ${catkin_EXPORTED_TARGETS})

# lcd_board_node
add_executable(lcd_board_node src/lcd_board_node.cpp include/peripheral_manager.hpp)
target_link_libraries(lcd_board_node lcd_board    ${catkin_LIBRARIES})

# motorcontroller
add_library(motorcontroller src/motorcontroller.cpp )
target_link_libraries(motorcontroller ${catkin_LIBRARIES})
add_dependencies(motorcontroller   ${catkin_EXPORTED_TARGETS})

# motorcontroller_node
add_executable(motorcontroller_node src/motorcontroller_node.cpp include/peripheral_manager.hpp)
target_link_libraries(motorcontroller_node motorcontroller    ${catkin_LIBRARIES})

# powerboard
add_library(powerboard src/powerboard.cpp )
target_link_libraries(powerboard ${catkin_LIBRARIES})
add_dependencies(powerboard   ${catkin_EXPORTED_TARGETS})

# powerboard_node
add_executable(powerboard_node src/powerboard_node.cpp include/peripheral_manager.hpp)
target_link_libraries(powerboard_node powerboard    ${catkin_LIBRARIES})

# torpedo
add_library(torpedo src/torpedo.cpp )
target_link_libraries(torpedo ${catkin_LIBRARIES})
add_dependencies(torpedo   ${catkin_EXPORTED_TARGETS})

# torpedo_node
add_executable(torpedo_node src/torpedo_node.cpp include/peripheral_manager.hpp)
target_link_libraries(torpedo_node torpedo    ${catkin_LIBRARIES})

# Sailorman
add_library(sailorman src/sailorman.cpp )
target_link_libraries(sailorman ${catkin_LIBRARIES})
add_dependencies(sailorman ${catkin_EXPORTED_TARGETS})

# Sailorman_node
add_executable(sailorman_node src/sailorman_node include/sailorman.hpp)
target_link_libraries(sailorman_node sailorman ${catkin_LIBRARIES})

# periodic_node
add_executable(periodic_node src/periodic_node include/sailorman.hpp)
target_link_libraries(periodic_node sailorman ${catkin_LIBRARIES})

install(
  TARGETS
    dropper
    dropper_node
    dvl
    dvl_node
    grabber
    grabber_node
    hydrophone
    hydrophone_node
    imu
    imu_node
    lcd_board
    lcd_board_node
    motorcontroller
    motorcontroller_node
    powerboard
    powerboard_node
    torpedo
    torpedo_node
    sailorman
    sailorman_node
    periodic_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
if(CATKIN_ENABLE_TESTING)
  # add gtest ased cpp test target and link LIBRARIES
  #catkin_add_gtest(Test_SailorMan test/TEST_SailorMan.cpp include/sailorman.hpp)
  #target_link_libraries(Test_SailorMan Sailorman ${catkin_LIBRARIES})

endif()

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)