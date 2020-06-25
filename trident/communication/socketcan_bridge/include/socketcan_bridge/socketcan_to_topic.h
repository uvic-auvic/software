
#ifndef SOCKETCAN_BRIDGE_SOCKETCAN_TO_TOPIC_H
#define SOCKETCAN_BRIDGE_SOCKETCAN_TO_TOPIC_H

#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/filter.h>
#include <can_msgs/Frame.h>
#include <ros/ros.h>

namespace socketcan_bridge
{
class SocketCANToTopic
{
  public:
    SocketCANToTopic(ros::NodeHandle* nh, ros::NodeHandle* nh_param, can::DriverInterfaceSharedPtr driver);
    void setup();
    void setup(const can::FilteredFrameListener::FilterVector &filters);
    void setup(XmlRpc::XmlRpcValue filters);
    void setup(ros::NodeHandle nh);

  private:
    ros::Publisher can_topic_;
    can::DriverInterfaceSharedPtr driver_;

    can::FrameListenerConstSharedPtr frame_listener_;
    can::StateListenerConstSharedPtr state_listener_;


    void frameCallback(const can::Frame& f);
    void stateCallback(const can::State & s);
};

void convertSocketCANToMessage(const can::Frame& f, can_msgs::Frame& m)
{
  m.id = f.id;
  m.dlc = f.dlc;
  m.is_error = f.is_error;
  m.is_rtr = f.is_rtr;
  m.is_extended = f.is_extended;

  for (int i = 0; i < 8; i++)  // always copy all data, regardless of dlc.
  {
    m.data[i] = f.data[i];
  }
};

};  // namespace socketcan_bridge


#endif  // SOCKETCAN_BRIDGE_SOCKETCAN_TO_TOPIC_H
