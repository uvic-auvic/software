cmake_minimum_required(VERSION 3.0.2)
project(auvic_msgs)

find_package(catkin REQUIRED
  can_msgs 
  message_generation 
  std_msgs 
  geometry_msgs 
)
add_service_files(
  FILES
  devices_MC_getMotorEnums.srv
  devices_MC_motor.srv
  devices_MC_motors.srv
  devices_PB_avgData.srv
  devices_PB_powerEnable.srv
  devices_to_monitor.srv

)

add_message_files(
  FILES
  protocol_allMessages_U.msg
  protocol_deviceName_S.msg
  protocol_ISOTP_S.msg
  protocol_MCMessageRequest_S.msg
  protocol_MID_E.msg
  protocol_motorRPM_S.msg
  protocol_motorSetSpeed_S.msg
  protocol_PBBattCurrents_S.msg
  protocol_PBBattVoltages_S.msg
  protocol_PBEnvData_S.msg
  protocol_PBMessageRequest_S.msg
  protocol_powerEnable_S.msg
  devices_MC_motorEnums.msg
  devices_MC_rpms.msg
  devices_PB_powerboardData.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)