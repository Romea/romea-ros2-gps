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
#include <cmath>

// ros
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

// romea
#include "romea_core_common/time/Time.hpp"
#include "romea_core_common/geodesy/LambertConverter.hpp"
#include "romea_common_utils/params/sensor_parameters.hpp"
#include "romea_common_utils/qos.hpp"

// local
#include "romea_gps_utils/gps_data_conversions.hpp"
#include "romea_gps_utils/gps_data_diagnostics.hpp"
#include "romea_gps_utils/gps_data.hpp"

namespace romea::ros2
{

//-----------------------------------------------------------------------------
GpsData::GpsData(std::shared_ptr<rclcpp::Node> node)
: clock_(node->get_clock()),
  timer_(nullptr),
  diagnostics_(nullptr),
  fix_publisher_(nullptr),
  vel_publisher_(nullptr),
  nmea_sentence_publisher_(nullptr),
  diagnostics_publisher_(nullptr)
{
  declare_rate(node);
  declare_frame_id(node);
  init_publishers_(node);
  init_diagnostics_(node);
  init_timer_(node);
}

//-----------------------------------------------------------------------------
void GpsData::init_diagnostics_(std::shared_ptr<rclcpp::Node> node)
{
  diagnostics_ = std::make_unique<GpsDataDiagnostics>(get_rate(node));

  diagnostics_publisher_ = make_diagnostic_publisher<core::DiagnosticReport>(
    node, "gps", 1., "", "/diagnostics", true);
}

//-----------------------------------------------------------------------------
void GpsData::init_publishers_(std::shared_ptr<rclcpp::Node> node)
{
  auto frame_id = get_frame_id(node);

  fix_publisher_ = make_stamped_data_publisher<core::GGAFrame, sensor_msgs::msg::NavSatFix>(
    node, "fix", frame_id, sensor_data_qos(), true);
  vel_publisher_ = make_stamped_data_publisher<core::RMCFrame, geometry_msgs::msg::TwistStamped>(
    node, "vel", frame_id, sensor_data_qos(), true);
  nmea_sentence_publisher_ = make_stamped_data_publisher<std::string, nmea_msgs::msg::Sentence>(
    node, "nmea", frame_id, best_effort(30), true);
}

//-----------------------------------------------------------------------------
void GpsData::init_timer_(std::shared_ptr<rclcpp::Node> node)
{
  auto callback = std::bind(&GpsData::timer_callback_, this);
  timer_ = node->create_wall_timer(core::durationFromSecond(0.1), callback);
}

//-----------------------------------------------------------------------------
bool GpsData::can_be_converted_to_fix_msg_(const core::GGAFrame & gga_frame)
{
  return gga_frame.latitude &&
         gga_frame.longitude &&
         gga_frame.altitudeAboveGeoid &&
         gga_frame.geoidHeight &&
         gga_frame.horizontalDilutionOfPrecision &&
         gga_frame.fixQuality;
}

//-----------------------------------------------------------------------------
bool GpsData::can_be_converted_to_vel_msg_(const core::RMCFrame & rmc_frame)
{
  return rmc_frame.speedOverGroundInMeterPerSecond &&
         rmc_frame.trackAngleTrue;
}

//-----------------------------------------------------------------------------
void GpsData::process_gga_frame_(
  const rclcpp::Time & stamp,
  const std::string & nmea_sentence)
{
  core::GGAFrame gga_frame(nmea_sentence);
  diagnostics_->updateGGARate(to_romea_duration(stamp));
  if (can_be_converted_to_fix_msg_(gga_frame)) {
    // TODO(Jean) use EURE
    fix_publisher_->publish(stamp, gga_frame);
  }
}

//-----------------------------------------------------------------------------
void GpsData::process_rmc_frame_(
  const rclcpp::Time & stamp,
  const std::string & nmea_sentence)
{
  core::RMCFrame rmc_frame(nmea_sentence);
  diagnostics_->updateRMCRate(to_romea_duration(stamp));
  if (can_be_converted_to_vel_msg_(rmc_frame)) {
    vel_publisher_->publish(stamp, rmc_frame);
  }
}

//-----------------------------------------------------------------------------
void GpsData::process_gsv_frame_(
  const rclcpp::Time & stamp,
  const std::string & nmea_sentence)
{
  core::GSVFrame gsv_frame(nmea_sentence);
  if (gsv_frame.sentenceNumber == gsv_frame.numberOfSentences) {
    diagnostics_->updateGSVRate(to_romea_duration(stamp));
  }
}

//-----------------------------------------------------------------------------
void GpsData::process_nmea_sentence(const std::string & nmea_sentence)
{
  rclcpp::Time stamp = clock_->now();
  nmea_sentence_publisher_->publish(stamp, nmea_sentence);

  switch (core::NMEAParsing::extractSentenceId(nmea_sentence)) {
    case core::NMEAParsing::SentenceID::GGA:
      process_gga_frame_(stamp, nmea_sentence);
      break;
    case core::NMEAParsing::SentenceID::RMC:
      process_rmc_frame_(stamp, nmea_sentence);
      break;
    case core::NMEAParsing::SentenceID::GSV:
      process_gsv_frame_(stamp, nmea_sentence);
      break;
    default:
      break;
  }
}

//-----------------------------------------------------------------------------
void GpsData::timer_callback_()
{
  auto stamp = clock_->now();
  diagnostics_publisher_->publish(stamp, diagnostics_->makeReport(to_romea_duration(stamp)));
}

}  // namespace romea::ros2
