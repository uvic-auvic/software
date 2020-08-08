#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <string>
#include <auvic_msgs/Monitor_Jetson_To_World.h>
#include <can_msgs/Frame.h>

// Helper methods to create message
auto createCANMessage = [](uint32_t id, 
                            bool is_extended, 
                            bool is_rtr, 
                            bool is_error, 
                            int dlc, 
                            uint8_t data[8]
                            ){
    can_msgs::Frame msg;
    msg.id = id;
    msg.dlc = dlc;
    msg.is_error = is_error;
    msg.is_extended = is_extended;
    msg.is_rtr = is_rtr;
    for(int i = 0; i < 8; i++){
        msg.data[i] = data[i];
    }
    return msg;
};
auto createMessage = [](std::string msg_name, can_msgs::Frame msg){
    auvic_msgs::Monitor_Jetson_To_World frame;
    frame.message_name.data = msg_name;
    frame.CAN_MSG = msg;
    return frame;
};


class SH{
public:
    SH(){};
    ~SH(){};
    void subscriber_callback(const can_msgs::Frame::ConstPtr& msg){
        ROS_INFO("Callback is triggered.");

        //callback_data = createCANMessage(msg->id, msg->is_extended, msg->is_rtr, msg->is_error, msg->dlc, msg->data);
    };
    can_msgs::Frame callback_data;
};

class SHFixture :public::testing::Test{
protected:
    auvic_msgs::Monitor_Jetson_To_World msg;
    can_msgs::Frame can;
    std::string topic = "protocol_MID_TRIDENT_deviceName";
    uint8_t payload[8] = {0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01};
    uint32_t id{1};
    bool is_extended{false}, is_rtr{false}, is_error{false};
    int dlc{1};

    SHFixture(){
        can = createCANMessage(id, is_extended, is_rtr, is_error, dlc, payload);
        msg = createMessage(topic, can);
    }
};
// this test covers both publishing and subscribing of JetsonToWOrld
// TEST_F(SHFixture, TestJetsonToWorld){
//         // first is for publisher, second for subscriber. the namespace matters
//         ros::NodeHandle nh("protocols"), n("");
//         SH helper();
//         // for testing SH susbscribing
//         ros::Publisher pub = nh.advertise<auvic_msgs::Monitor_Jetson_To_World>(topic, 10);
//         // for testing SH publishing
//         ros::Subscriber sub = n.subscribe<can_msgs::Frame>("sent_messages", 10, &SH::subscriber_callback, &helper);
//         EXPECT_EQ(pub.getNumSubscribers(), 1U);
//         EXPECT_EQ(sub.getNumPublishers(), 1U);
//         pub.publish(msg);
//         // make a ros spin so our callback gets called.
//         ros::spinOnce();
//         EXPECT_EQ(can, helper.callback_data);
//         // if callback_data and can are equal, then we know original message was correctly
//         // subscribed by jetsonToWorld
// };


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "test_node_SH");

    return RUN_ALL_TESTS();
};