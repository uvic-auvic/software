#include <ros/ros.h>
#include <gtest/gtest.h>

TEST(TestSuite, test1)
{
  EXPECT_EQ(1,1);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ai_test");
  return RUN_ALL_TESTS();
}
