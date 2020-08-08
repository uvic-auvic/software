# Communication

This folder contains the packages required for the network layer setup between the jetson and ECUs (Electronic Controller Units).

## SocketCAN_Interface & SocketCAN_Bridge
Taken from the ros_canopen package. These two packages link the open source CAN drivers, socketCAN, to ROS. It provides two topics, sent_messages and received_messages, to send and receive from the CAN bus. It uses the can_msgs:Frame. Currently set to the virtual CAN bus, vcan0, but can easily change to the physical bus by modifying the roslaunch file and adding -DDRIVER=socket to catkin_make.

## can_msgs
Taken from the ros_canopen package. Contains the message "Frame" that is used to receive/send messages from/to the CAN bus.

## Monitor
Contains the interface between the "protocol" topics and socketcan_bridge. This package:
- Reads the received_messages topic via HardwareToSoftware_node and publishes them to the topic listed in its message ID.
- Sends messages to the sent_messages topic via SoftwareToHardware_node received from the protocol topics.
- NOTE: python tests are placed in src; c++ tests are placed in a test folder.

**Insert Picture of rqt_graph here**