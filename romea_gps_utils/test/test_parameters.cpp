// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license


// std
#include <string>
#include <memory>

// gtest
#include "gtest/gtest.h"

// local
#include "../test/test_helper.h"
#include "romea_gps_utils/gps_parameters.hpp"

class TestGPSParameters : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    rclcpp::NodeOptions no;

    no.arguments(
      {"--ros-args",
        "--params-file",
        std::string(TEST_DIR) + "/test_parameters.yaml"});

    node = std::make_shared<rclcpp::Node>("test_gps_parameters", "ns", no);
  }

  std::shared_ptr<rclcpp::Node> node;
};

TEST_F(TestGPSParameters, get_gps_fix_eure) {
  romea::declare_gps_gps_fix_eure(node);
  EXPECT_DOUBLE_EQ(romea::get_gps_gps_fix_eure(node), 3.0);
}

TEST_F(TestGPSParameters, get_dgps_fix_eure) {
  romea::declare_gps_dgps_fix_eure(node);
  EXPECT_DOUBLE_EQ(romea::get_gps_dgps_fix_eure(node), 1.0);
}

TEST_F(TestGPSParameters, get_float_rtk_fix_eure) {
  romea::declare_gps_float_rtk_fix_eure(node);
  EXPECT_DOUBLE_EQ(romea::get_gps_float_rtk_fix_eure(node), 0.5);
}

TEST_F(TestGPSParameters, get_rtk_fix_eure) {
  romea::declare_gps_rtk_fix_eure(node);
  EXPECT_DOUBLE_EQ(romea::get_gps_rtk_fix_eure(node), 0.1);
}

TEST_F(TestGPSParameters, get_simulation_fix_eure) {
  romea::declare_gps_simulation_fix_eure(node);
  EXPECT_DOUBLE_EQ(romea::get_gps_simulation_fix_eure(node), 0.02);
}

TEST_F(TestGPSParameters, get_gps_antenna_body_position) {
  romea::declare_gps_antenna_body_position(node);
  auto position = romea::get_gps_antenna_body_position(node);
  EXPECT_DOUBLE_EQ(position[0], 1.0);
  EXPECT_DOUBLE_EQ(position[1], 2.0);
  EXPECT_DOUBLE_EQ(position[2], 3.0);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
