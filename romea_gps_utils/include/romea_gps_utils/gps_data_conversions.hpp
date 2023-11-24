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


#ifndef ROMEA_GPS_UTILS__GPS_DATA_CONVERSIONS_HPP_
#define ROMEA_GPS_UTILS__GPS_DATA_CONVERSIONS_HPP_

// std
#include <string>

// ros
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nmea_msgs/msg/sentence.hpp"

// romea core
#include "romea_core_gps/GPSReceiverEUREs.hpp"
#include "romea_core_gps/nmea/NMEAParsing.hpp"
#include "romea_core_gps/nmea/GGAFrame.hpp"
#include "romea_core_gps/nmea/RMCFrame.hpp"
#include "romea_core_gps/nmea/GSVFrame.hpp"


namespace romea
{
namespace ros2
{

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const std::string & raw_sentence,
  nmea_msgs::msg::Sentence & msg);

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const core::GGAFrame & gga_frame,
  sensor_msgs::msg::NavSatFix & msg);

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const core::RMCFrame & rmc_frame,
  geometry_msgs::msg::TwistStamped & msg);


}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_GPS_UTILS__GPS_DATA_CONVERSIONS_HPP_
