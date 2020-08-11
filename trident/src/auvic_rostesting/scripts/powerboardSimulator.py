#!/usr/bin/env python
import sys
import can
import rospy

bus = can.interface.Bus(bustype='socketcan',channel='vcan0',bitrate=500000)


def timer_callback_powerboardID(event):
    msg = can.Message(arbitration_id=41,is_extended_id=False)
    bus.send(msg)

def PB_sim():
    rospy.init_node('powerboardSimulator', anonymous=True)
    rate = rospy.Rate(10000) # 10khz
    rospy.Timer(rospy.Duration(1), timer_callback_powerboardID, oneshot=False)
    # listen to the CAN bus and publish device ID every second.
    while not rospy.is_shutdown():
        for message in bus:
            # This node will detect its own sent messages, so ignore them.
            if message.arbitration_id == 41:
                pass
            else:
                rospy.loginfo("AUVIC_ROSTESTING/PB_SIM: received message ID " + str(message.arbitration_id))
                if message.arbitration_id == 1: # the device ID for POLARIS
                    print("ID: ",message.data)
                # end if
            # end else
        # end for
    #end while    
    rate.sleep()


if __name__ == '__main__':
    try:
        PB_sim()
    except rospy.ROSInterruptException:
        pass