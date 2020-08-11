#!/usr/bin/env python
PKG = 'monitor'
import sys
import unittest
from can_msgs import Frame 

class rospyObj:
    def __init__(self, name):
        self.name = name
        rospy.init_node(name, anonymous=True)
        rate = rospy.Rate(10000) # 10khz
        try:
            self.rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")

    def __del__(self):
        rospy.signal_shutdown("Done testing with " + self.name)
    
    def callback(self):

    def make_subscriber(self,topic)
        self.sub = rospy.Subscriber(topic, Frame, self.callback)

class TestHS(unittest.TestCase):
    def setUp(self):
        self.node = rospyObj("test_HS")
        
    def tearDown(self):
        pass
    # test if worldToJetson correctly publishes to topic protocol_MID_TRIDENT_deviceName
    def test_HS_worldToJetson_pub(self):
        pass


    # test if worldToJetson correctly subscribes to topic received_messages
    def test_HS_worldToJetson_sub(self):
        msg = Frame()
        msg.id = 0x29
        msg.data[0] = 0x00
        pass

if __name__ = '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_node_HS', TestHS)