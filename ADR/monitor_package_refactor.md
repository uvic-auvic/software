# ADR: Monitor package Refactor from UART to CAN protocol

## Status
decided

## Timeline 
June. 18, 2020: ADR published

## Context
The focus of this ADR is the ROS monitor package. Specifically, on how the Jetson communicates with the embedded system. AUVICs electrical team wishes to use CAN to have every device share a single communication bus. The can bus will remove the need for a "host" device as each device will be assigned an ID based on priority, this will allow the Jetson to broadcast messages to either specific or all devices as needed.

## Decision
- The decision was made becuase our current communciation method is not optimized for multiple devices. With the CAN bus, we can decentralize the communication and have each device communicate with one another without needing the Jetson as an arbitrator.

## Consequences 
- Each device will need its own Object dictionary. This is done using the vector software, CANeds.
- The CAN bus works by giving priority to the device with the lowest COB-ID - a value that is assigned during configuration - and filtering the frames using bit-wise aribitration. 

## Comments
_Relevant discussions and decisions may be recorded in this section for context_

## Links
