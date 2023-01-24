// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

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

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const std::string & raw_sentence,
  nmea_msgs::msg::Sentence & msg);

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const GGAFrame & gga_frame,
  sensor_msgs::msg::NavSatFix & msg);

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const RMCFrame & rmc_frame,
  geometry_msgs::msg::TwistStamped & msg);


}  // namespace romea

#endif  // ROMEA_GPS_UTILS__GPS_DATA_CONVERSIONS_HPP_
