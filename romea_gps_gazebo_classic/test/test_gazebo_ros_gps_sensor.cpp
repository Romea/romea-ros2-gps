// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// std
#include <memory>
#include <string>

// gazebo
#include "gazebo/test/ServerFixture.hh"
#include "gazebo_ros/node.hpp"
#include "gazebo_ros/testing_utils.hpp"

// ros
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nmea_msgs/msg/sentence.hpp"

// romea core
#include "romea_core_gps/nmea/NMEAParsing.hpp"
#include "romea_core_gps/nmea/GGAFrame.hpp"
#include "romea_core_gps/nmea/RMCFrame.hpp"


// local
#include "../test/test_helper.h"


#define tol 10e-4

class NmeaListener
{
public:
  NmeaListener()
  : node_(std::make_shared<rclcpp::Node>("test_gazebo_ros_gps_sensor")),
    nmea_sub_(),
    gga_frame_(),
    rmc_frame_()
  {
    auto callback = std::bind(&NmeaListener::listen_nmea_, this, std::placeholders::_1);
    auto qos = rclcpp::QoS(rclcpp::KeepLast(2)).best_effort().durability_volatile();
    nmea_sub_ = node_->create_subscription<nmea_msgs::msg::Sentence>(
      "/gps/nmea_sentence", qos, callback);
  }

  void spin_some()
  {
    rclcpp::spin_some(node_);
  }

  std::optional<romea::core::GGAFrame> get_gga_frame() {return gga_frame_.value();}
  std::optional<romea::core::RMCFrame> get_rmc_frame() {return rmc_frame_.value();}
  bool ok() {return gga_frame_.has_value() && rmc_frame_.has_value();}
  void reset() {gga_frame_.reset(); rmc_frame_.reset();}

private:
  void listen_nmea_(nmea_msgs::msg::Sentence::ConstSharedPtr msg)
  {
    std::cout << msg->sentence << std::endl;
    switch (romea::core::NMEAParsing::extractSentenceId(msg->sentence)) {
      case romea::core::NMEAParsing::SentenceID::GGA:
        gga_frame_ = romea::core::GGAFrame(msg->sentence);
        break;
      case romea::core::NMEAParsing::SentenceID::RMC:
        rmc_frame_ = romea::core::RMCFrame(msg->sentence);
        break;
      default:
        break;
    }
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Subscription<nmea_msgs::msg::Sentence>> nmea_sub_;

  std::optional<romea::core::GGAFrame> gga_frame_;
  std::optional<romea::core::RMCFrame> rmc_frame_;
};

class NavSatListener
{
public:
  NavSatListener()
  : node_(std::make_shared<rclcpp::Node>("test_gazebo_ros_gps_sensor")),
    fix_sub_(),
    vel_sub_(),
    fix_msg_(),
    vel_msg_()
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

    auto fix_callback = std::bind(&NavSatListener::listen_fix_, this, std::placeholders::_1);
    fix_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", qos, fix_callback);

    auto vel_callback = std::bind(&NavSatListener::listen_vel_, this, std::placeholders::_1);
    vel_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/gps/vel", qos, vel_callback);
  }

  void spin_some()
  {
    rclcpp::spin_some(node_);
  }

  std::optional<sensor_msgs::msg::NavSatFix> get_fix_msg() {return fix_msg_.value();}
  std::optional<geometry_msgs::msg::TwistStamped> get_vel_msg() {return vel_msg_.value();}
  bool ok() {return fix_msg_.has_value() && vel_msg_.has_value();}
  void reset() {fix_msg_.reset(); vel_msg_.reset();}

private:
  void listen_fix_(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
  {
    fix_msg_ = *msg;
  }

  void listen_vel_(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    vel_msg_ = *msg;
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::NavSatFix>> fix_sub_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>> vel_sub_;

  std::optional<sensor_msgs::msg::NavSatFix> fix_msg_;
  std::optional<geometry_msgs::msg::TwistStamped> vel_msg_;
};


/// Tests the gazebo_ros_gps_sensor plugin
class GazeboRosGpsSensorTest : public gazebo::ServerFixture
{
public:
  void load_world()
  {
    // Load test world and start paused
    auto world_name = std::string(TEST_DIR) +
      std::string("/test_gazebo_ros_gps_sensor.world");

    this->Load(world_name, true);

    // World
    world = gazebo::physics::get_world();
    ASSERT_NE(nullptr, world);

    // Get the model with the attached GPS
    box = world->ModelByName("box");
    link = box->GetLink("link");
    ASSERT_NE(nullptr, box);

    // Make sure sensor is loaded
    ASSERT_EQ(link->GetSensorCount(), 1u);

    world->Step(1);
    EXPECT_NEAR(0.0, box->WorldPose().Pos().X(), tol);
    EXPECT_NEAR(0.0, box->WorldPose().Pos().Y(), tol);
    EXPECT_NEAR(0.5, box->WorldPose().Pos().Z(), tol);
  }

  template<typename Listener>
  void move_and_listen(
    Listener & listener,
    double x,
    double y,
    double z,
    double vx,
    double vy,
    double vz)
  {
    ignition::math::Pose3d box_pose;
    box_pose.Pos() = {x, y, z};
    link->SetWorldPose(box_pose);
    link->SetWorldTwist({vx, vy, vz}, {0, 0, 0});

    int sleep = 0;
    int max_sleep{500};
    double zb = 1000.0;

    do {
      world->Step(10);
      listener.spin_some();
      gazebo::common::Time::MSleep(10);
      sleep++;
      std::cout << sleep << std::endl;
      std::cout << box->WorldPose().Pos().X() << " ";
      std::cout << box->WorldPose().Pos().Y() << " ";
      std::cout << box->WorldPose().Pos().Z() << std::endl;
      std::cout << link->WorldLinearVel().X() << " ";
      std::cout << link->WorldLinearVel().Y() << " ";
      std::cout << link->WorldLinearVel().Z() << std::endl;
      zb = box->WorldPose().Pos().Z();
    }  while (sleep < max_sleep && !(listener.ok() && std::abs(zb - z) < 1));

    ASSERT_LT(sleep, max_sleep);
  }

  gazebo::physics::WorldPtr world;
  gazebo::physics::ModelPtr box;
  gazebo::physics::LinkPtr link;
};

TEST_F(GazeboRosGpsSensorTest, checkNmeaMessagesAtRest)
{
  load_world();
  NmeaListener nmea_listener;
  move_and_listen(nmea_listener, 0, 0, 0, 0, 0, 0);

  auto gga_frame = nmea_listener.get_gga_frame();
  ASSERT_TRUE(gga_frame.has_value());
  EXPECT_NEAR(3.0, gga_frame->longitude->toDouble() * 180 / M_PI, tol);
  EXPECT_NEAR(45.0, gga_frame->latitude->toDouble() * 180 / M_PI, tol);
  EXPECT_NEAR(0.5, *gga_frame->altitudeAboveGeoid, tol);
  EXPECT_NEAR(1.0, *gga_frame->horizontalDilutionOfPrecision, tol);
  EXPECT_EQ(*gga_frame->fixQuality, romea::core::FixQuality::SIMULATION_FIX);

  auto rmc_frame = nmea_listener.get_rmc_frame();
  ASSERT_TRUE(rmc_frame.has_value());
  EXPECT_NEAR(3.0, rmc_frame->longitude->toDouble() * 180 / M_PI, tol);
  EXPECT_NEAR(45.0, rmc_frame->latitude->toDouble() * 180 / M_PI, tol);
  EXPECT_NEAR(0.0, *rmc_frame->speedOverGroundInMeterPerSecond, tol);
}

TEST_F(GazeboRosGpsSensorTest, checkNmeaMessagesInMovement)
{
  load_world();
  NmeaListener nmea_listener;
  move_and_listen(nmea_listener, 100.0, 200.0, 300.0, 1.0, 2.0, 3.0);

  auto gga_frame = nmea_listener.get_gga_frame();
  ASSERT_TRUE(gga_frame.has_value());
  EXPECT_GT(gga_frame->longitude->toDouble() * 180 / M_PI, 3.0);
  EXPECT_GT(gga_frame->latitude->toDouble() * 180 / M_PI, 45.0);
  EXPECT_NEAR(*gga_frame->altitudeAboveGeoid, 300, 1);
  EXPECT_NEAR(*gga_frame->horizontalDilutionOfPrecision, 1.0, tol);
  EXPECT_EQ(*gga_frame->fixQuality, romea::core::FixQuality::SIMULATION_FIX);

  auto rmc_frame = nmea_listener.get_rmc_frame();
  ASSERT_TRUE(rmc_frame.has_value());
  EXPECT_GT(rmc_frame->longitude->toDouble() * 180 / M_PI, 3.0);
  EXPECT_GT(rmc_frame->latitude->toDouble() * 180 / M_PI, 45.0);
  EXPECT_NEAR(*rmc_frame->speedOverGroundInMeterPerSecond, std::sqrt(5), 0.1);
  EXPECT_NEAR(*rmc_frame->trackAngleTrue, M_PI_2 - std::atan(2), 0.1);
}


TEST_F(GazeboRosGpsSensorTest, CheckNavSatMessagesAtRest)
{
  load_world();
  NavSatListener nav_sat_listener;
  move_and_listen(nav_sat_listener, 0, 0, 0, 0, 0, 0);

  auto fix_msg = nav_sat_listener.get_fix_msg();
  ASSERT_TRUE(fix_msg.has_value());
  EXPECT_NEAR(3.0, fix_msg->longitude, tol);
  EXPECT_NEAR(45.0, fix_msg->latitude, tol);
  EXPECT_NEAR(0.5, fix_msg->altitude, tol);

  auto vel_msg = nav_sat_listener.get_vel_msg();
  ASSERT_TRUE(vel_msg.has_value());
  EXPECT_NEAR(0.0, vel_msg->twist.linear.x, tol);
  EXPECT_NEAR(0.0, vel_msg->twist.linear.y, tol);
  EXPECT_NEAR(0.0, vel_msg->twist.linear.z, tol);
}

TEST_F(GazeboRosGpsSensorTest, CheckNavSatMessagesAtMovement)
{
  load_world();
  NavSatListener nav_sat_listener;
  move_and_listen(nav_sat_listener, 100.0, 200.0, 300.0, 1.0, 2.0, 3.0);

  auto fix_msg = nav_sat_listener.get_fix_msg();
  ASSERT_TRUE(fix_msg.has_value());
  EXPECT_GT(fix_msg->longitude, 3.0);
  EXPECT_GT(fix_msg->latitude, 45.0);
  EXPECT_NEAR(fix_msg->altitude, 300, 1);

  auto vel_msg = nav_sat_listener.get_vel_msg();
  ASSERT_TRUE(vel_msg.has_value());
  EXPECT_NEAR(vel_msg->twist.linear.x, 1.0, tol);
  EXPECT_NEAR(vel_msg->twist.linear.y, 2.0, tol);
  EXPECT_NEAR(vel_msg->twist.linear.z, 3.0, 1);  // why z vel is not equal to sepoint ?
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
