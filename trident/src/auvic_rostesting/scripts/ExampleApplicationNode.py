#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
from auvic_msgs.srv import Monitor_Send_Data_To_World
from auvic_msgs.msg import *
from can_msgs.msg import Frame

def main(args=None):
    #FUUUUUUUUUUUUUUUU
    # Finaly. with rospy, you need to include the namespace with the service, if there is any. unlike C++, that does it under the hood.
    # Hoenstly Just stick with C++. THeres so much more information out there.
    rospy.wait_for_service('Send_Data_To_World')
    try:
        Send_Data_To_World = rospy.ServiceProxy('Send_Data_To_World', Monitor_Send_Data_To_World)

        can = Frame()
        can.header.seq = 1
        can.header.frame_id = "22"
        can.id = 1 # trident's message ID is 1
        can.is_error = False
        can.is_rtr = False
        can.is_extended = False
        can.dlc = 2
        can.data = [0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11]

        letter = Monitor_Jetson_To_World()
        letter.message_name.data = "protocol_MID_TRIDENT_deviceName"
        letter.CAN_MSG = can

        resp = Send_Data_To_World(letter)
        print(resp.output.data)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == '__main__':
    main()