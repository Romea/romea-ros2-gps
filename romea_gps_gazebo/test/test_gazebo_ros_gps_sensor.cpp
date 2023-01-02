// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// ros
#include <gazebo/test/ServerFixture.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/testing_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

// romea core
#include <romea_core_gps/nmea/GGAFrame.hpp>
#include <romea_core_gps/nmea/RMCFrame.hpp>

// std
#include <memory>

#define tol 10e-4

/// Tests the gazebo_ros_gps_sensor plugin
class GazeboRosGpsSensorTest : public gazebo::ServerFixture
{
};

TEST_F(GazeboRosGpsSensorTest, checkNavsatMessages)
{
  // Load test world and start paused
  this->Load("test_gazebo_ros_gps_sensor.world", true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Get the model with the attached GPS
  auto box = world->ModelByName("box");
  auto link = box->GetLink("link");
  ASSERT_NE(nullptr, box);

  // Make sure sensor is loaded
  ASSERT_EQ(link->GetSensorCount(), 1u);

  // Create node / executor for receiving gps message
  auto node = std::make_shared<rclcpp::Node>("test_gazebo_ros_gps_sensor");
  ASSERT_NE(nullptr, node);

  sensor_msgs::msg::NavSatFix::SharedPtr nmea_sentence_msg = nullptr;
  std::shared_ptr<romea::GGAFrame> gga_frame = nullptr;
  std::shared_ptr<romea::RMCFrame> rmc_frame = nullptr;

  auto fix_sub = node->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/gps/nmea_sentence", rclcpp::SensorDataQoS(),
    [&nmea_sentence_msg](sensor_msgs::msg::NavSatFix::SharedPtr _msg) {
      nmea_sentence_msg = _msg;
    });

  world->Step(1);
  EXPECT_NEAR(0.0, box->WorldPose().Pos().X(), tol);
  EXPECT_NEAR(0.0, box->WorldPose().Pos().Y(), tol);
  EXPECT_NEAR(0.5, box->WorldPose().Pos().Z(), tol);

  // Step until a gps message will have been published
  int sleep{0};
  int max_sleep{1000};
  do {
    world->Step(100);
    rclcpp::spin_some(node);
    gazebo::common::Time::MSleep(100);
    sleep++;

    if (nmea_sentence_msg != nullptr) {

    }
  }  while (sleep < max_sleep && nullptr == fix_msg);

//  EXPECT_LT(0u, fix_sub->get_publisher_count());
//  EXPECT_LT(0u, vel_sub->get_publisher_count());
//  EXPECT_LT(sleep, max_sleep);
//  ASSERT_NE(nullptr, fix_msg);
//  ASSERT_NE(nullptr, vel_msg);

//  // Get the initial gps output when the box is at rest
//  auto pre_movement_fix_msg = std::make_shared<sensor_msgs::msg::NavSatFix>(*fix_msg);
//  ASSERT_NE(nullptr, pre_movement_fix_msg);
//  EXPECT_NEAR(0.0, pre_movement_fix_msg->latitude, tol);
//  EXPECT_NEAR(0.0, pre_movement_fix_msg->longitude, tol);
//  EXPECT_NEAR(0.5, pre_movement_fix_msg->altitude, tol);

//  auto pre_movement_vel_msg = std::make_shared<geometry_msgs::msg::TwistStamped>(*vel_msg);
//  ASSERT_NE(nullptr, pre_movement_vel_msg);
//  EXPECT_NEAR(0.0, pre_movement_vel_msg->twist.linear.x, tol);
//  EXPECT_NEAR(0.0, pre_movement_vel_msg->twist.linear.y, tol);
//  EXPECT_NEAR(0.0, pre_movement_vel_msg->twist.linear.z, tol);


//  // Change the position and linear speed of the link and step a few times to wait the ros message to be received
//  fix_msg = nullptr;
//  vel_msg = nullptr;
//  ignition::math::Pose3d box_pose;
//  box_pose.Pos() = {100.0, 200.0, 300.0};
//  link->SetWorldPose(box_pose);
//  link->SetWorldTwist({1,2,3},{0,0,0});

//  sleep = 0;
//  do{
//    world->Step(50);
//    rclcpp::spin_some(node);
//    gazebo::common::Time::MSleep(100);
//    sleep++;
//  }
//  while (sleep < max_sleep && (nullptr == fix_msg || fix_msg->altitude < 150));


//  // Check that GPS output reflects the position change
//  auto post_movement_fix_msg = std::make_shared<sensor_msgs::msg::NavSatFix>(*fix_msg);
//  ASSERT_NE(nullptr, post_movement_fix_msg);
//  EXPECT_GT(post_movement_fix_msg->latitude, 0.0);
//  EXPECT_GT(post_movement_fix_msg->longitude, 0.0);
//  EXPECT_NEAR(300, post_movement_fix_msg->altitude, 1);

//  auto post_movement_vel_msg = std::make_shared<geometry_msgs::msg::TwistStamped>(*vel_msg);
//  ASSERT_NE(nullptr, post_movement_vel_msg);
//  EXPECT_NEAR(post_movement_vel_msg->twist.linear.x, 1.0 ,10*tol);
//  EXPECT_NEAR(post_movement_vel_msg->twist.linear.y, 2.0 ,10*tol);
//  EXPECT_NEAR(post_movement_vel_msg->twist.linear.z, 3.0 ,10*tol); // why error is greater than tol ?
}


TEST_F(GazeboRosGpsSensorTest, CheckNmeaMessages)
{
  // Load test world and start paused
  this->Load("test_gazebo_ros_gps_sensor.world", true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Get the model with the attached GPS
  auto box = world->ModelByName("box");
  auto link = box->GetLink("link");
  ASSERT_NE(nullptr, box);

  // Make sure sensor is loaded
  ASSERT_EQ(link->GetSensorCount(), 1u);

  // Create node / executor for receiving gps message
  auto node = std::make_shared<rclcpp::Node>("test_gazebo_ros_gps_sensor");
  ASSERT_NE(nullptr, node);

  sensor_msgs::msg::NavSatFix::SharedPtr fix_msg = nullptr;
  auto fix_sub = node->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/gps/fix", rclcpp::SensorDataQoS(),
    [&fix_msg](sensor_msgs::msg::NavSatFix::SharedPtr _msg) {
      fix_msg = _msg;
    });

  geometry_msgs::msg::TwistStamped::SharedPtr vel_msg = nullptr;
  auto vel_sub = node->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/gps/vel", rclcpp::SensorDataQoS(),
    [&vel_msg](geometry_msgs::msg::TwistStamped::SharedPtr _msg) {
      vel_msg = _msg;
    });

  world->Step(1);
  EXPECT_NEAR(0.0, box->WorldPose().Pos().X(), tol);
  EXPECT_NEAR(0.0, box->WorldPose().Pos().Y(), tol);
  EXPECT_NEAR(0.5, box->WorldPose().Pos().Z(), tol);

  // Step until a gps message will have been published
  int sleep{0};
  int max_sleep{1000};
  do {
    world->Step(100);
    rclcpp::spin_some(node);
    gazebo::common::Time::MSleep(100);
    sleep++;
  }  while (sleep < max_sleep && nullptr == fix_msg);

  EXPECT_LT(0u, fix_sub->get_publisher_count());
  EXPECT_LT(0u, vel_sub->get_publisher_count());
  EXPECT_LT(sleep, max_sleep);
  ASSERT_NE(nullptr, fix_msg);
  ASSERT_NE(nullptr, vel_msg);

  // Get the initial gps output when the box is at rest
  auto pre_movement_fix_msg = std::make_shared<sensor_msgs::msg::NavSatFix>(*fix_msg);
  ASSERT_NE(nullptr, pre_movement_fix_msg);
  EXPECT_NEAR(0.0, pre_movement_fix_msg->latitude, tol);
  EXPECT_NEAR(0.0, pre_movement_fix_msg->longitude, tol);
  EXPECT_NEAR(0.5, pre_movement_fix_msg->altitude, tol);

  auto pre_movement_vel_msg = std::make_shared<geometry_msgs::msg::TwistStamped>(*vel_msg);
  ASSERT_NE(nullptr, pre_movement_vel_msg);
  EXPECT_NEAR(0.0, pre_movement_vel_msg->twist.linear.x, tol);
  EXPECT_NEAR(0.0, pre_movement_vel_msg->twist.linear.y, tol);
  EXPECT_NEAR(0.0, pre_movement_vel_msg->twist.linear.z, tol);


  // Change the position and linear speed of the link and step a few times to wait the ros message to be received
  fix_msg = nullptr;
  vel_msg = nullptr;
  ignition::math::Pose3d box_pose;
  box_pose.Pos() = {100.0, 200.0, 300.0};
  link->SetWorldPose(box_pose);
  link->SetWorldTwist({1, 2, 3}, {0, 0, 0});

  sleep = 0;
  do {
    world->Step(50);
    rclcpp::spin_some(node);
    gazebo::common::Time::MSleep(100);
    sleep++;
  } while (sleep < max_sleep && (nullptr == fix_msg || fix_msg->altitude < 150));


  // Check that GPS output reflects the position change
  auto post_movement_fix_msg = std::make_shared<sensor_msgs::msg::NavSatFix>(*fix_msg);
  ASSERT_NE(nullptr, post_movement_fix_msg);
  EXPECT_GT(post_movement_fix_msg->latitude, 0.0);
  EXPECT_GT(post_movement_fix_msg->longitude, 0.0);
  EXPECT_NEAR(300, post_movement_fix_msg->altitude, 1);

  auto post_movement_vel_msg = std::make_shared<geometry_msgs::msg::TwistStamped>(*vel_msg);
  ASSERT_NE(nullptr, post_movement_vel_msg);
  EXPECT_NEAR(post_movement_vel_msg->twist.linear.x, 1.0, 10 * tol);
  EXPECT_NEAR(post_movement_vel_msg->twist.linear.y, 2.0, 10 * tol);
  EXPECT_NEAR(post_movement_vel_msg->twist.linear.z, 3.0, 10 * tol); // why error is greater than tol ?

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
