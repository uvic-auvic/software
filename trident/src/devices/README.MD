# Devices

This package reads the protocol topics (can_msgs::Frame) and also publishes to them (auvic_msgs::Monitor_Jetson_To_World). There exists a node for every ECU on the can bus that is responsible to update their own data, stored as a static member of the parent class "Peripheral".

Information is collected from the parent class via service call to the "sailorman_node"; it has 2 jobs:
1. Send data to the CAN bus by service request on "Send_Data_To_World" server. Returns a "sent." or "dropped."
2. Retrieve data from the peripheral class via "Get_Data_From_Peripherals" server.
