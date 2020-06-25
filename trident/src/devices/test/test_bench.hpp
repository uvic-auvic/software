class TEST_Message_To_Specific_nodes: public Peripherals{
    public:
        TEST_Message_To_Specific_nodes(ros::NodeHandle* n_auvic);
        ~TEST_Message_To_Specific_nodes();
        bool topic_callback(const can_msgs::Frame::ConstPtr& msg);
        bool send_frame(const can_msgs::Frame::ConstPtr& msg);
};
