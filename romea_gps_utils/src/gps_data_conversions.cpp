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
#include <string>

// local
#include "romea_gps_utils/gps_data_conversions.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const std::string & raw_sentence,
  nmea_msgs::msg::Sentence & msg)
{
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.sentence = raw_sentence;
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const core::GGAFrame & gga_frame,
  sensor_msgs::msg::NavSatFix & msg)
{
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;

  msg.latitude = gga_frame.latitude->toDouble() * 180. / M_PI;
  msg.longitude = gga_frame.longitude->toDouble() * 180. / M_PI;
  msg.altitude = gga_frame.altitudeAboveGeoid.value() + gga_frame.geoidHeight.value();

  double hdop = gga_frame.horizontalDilutionOfPrecision.value();
  double std = hdop * core::GPSReceiverEUREs().get(*gga_frame.fixQuality);
  msg.position_covariance[0] = std * std;
  msg.position_covariance[4] = std * std;
  msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;


  core::FixQuality fix_quality = gga_frame.fixQuality.value();
  if (fix_quality == core::FixQuality::INVALID_FIX) {
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  } else if (fix_quality == core::FixQuality::GPS_FIX) {
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  } else if (fix_quality == core::FixQuality::DGPS_FIX) {
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
  } else if (fix_quality == core::FixQuality::FLOAT_RTK_FIX ||
    fix_quality == core::FixQuality::RTK_FIX)
  {
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
  }

  core::TalkerId talker_id = gga_frame.talkerId;

  if (talker_id == core::TalkerId::GP) {
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  } else if (talker_id == core::TalkerId::GL) {
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
  } else if (talker_id == core::TalkerId::GA) {
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
  } else if (talker_id == core::TalkerId::GB || talker_id == core::TalkerId::BD) {
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
  }
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const core::RMCFrame & rmc_frame,
  geometry_msgs::msg::TwistStamped & msg)
{
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;

  msg.twist.linear.x = rmc_frame.speedOverGroundInMeterPerSecond.value() *
    std::sin(rmc_frame.trackAngleTrue.value());
  msg.twist.linear.y = rmc_frame.speedOverGroundInMeterPerSecond.value() *
    std::cos(rmc_frame.trackAngleTrue.value());
}

}  // namespace ros2
}  // namespace romea
