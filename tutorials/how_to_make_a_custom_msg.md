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

when making a message containing custom types such as std_msgs and can_msgs, you need to specify types in the generate_messages(DEPENDENCIES ...) object.
