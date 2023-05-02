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

// romea core
#include "romea_core_common/time/Time.hpp"

// romea ros
#include "romea_common_utils/params/sensor_parameters.hpp"
#include "romea_common_utils/qos.hpp"

// local
#include "romea_gps_utils/gps_data_conversions.hpp"
#include "romea_gps_utils/gps_data_diagnostics.hpp"
#include "romea_gps_utils/gps_data.hpp"

#include "romea_core_common/geodesy/LambertConverter.hpp"

namespace romea
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

  diagnostics_publisher_ = make_diagnostic_publisher<DiagnosticReport>(
    node, "gps", 1., "", "/diagnostics", true);
}

//-----------------------------------------------------------------------------
void GpsData::init_publishers_(std::shared_ptr<rclcpp::Node> node)
{
  auto frame_id = get_frame_id(node);

  fix_publisher_ = make_stamped_data_publisher<GGAFrame, sensor_msgs::msg::NavSatFix>(
    node, "fix", frame_id, sensor_data_qos(), true);
  vel_publisher_ = make_stamped_data_publisher<RMCFrame, geometry_msgs::msg::TwistStamped>(
    node, "vel", frame_id, sensor_data_qos(), true);
  nmea_sentence_publisher_ = make_stamped_data_publisher<std::string, nmea_msgs::msg::Sentence>(
    node, "nmea", frame_id, sensor_data_qos(), true);
}

//-----------------------------------------------------------------------------
void GpsData::init_timer_(std::shared_ptr<rclcpp::Node> node)
{
  auto callback = std::bind(&GpsData::timer_callback_, this);
  timer_ = node->create_wall_timer(durationFromSecond(0.1), callback);
}

//-----------------------------------------------------------------------------
bool GpsData::can_be_converted_to_fix_msg_(const GGAFrame & gga_frame)
{
  return gga_frame.latitude &&
         gga_frame.longitude &&
         gga_frame.altitudeAboveGeoid &&
         gga_frame.geoidHeight &&
         gga_frame.horizontalDilutionOfPrecision &&
         gga_frame.fixQuality;
}

//-----------------------------------------------------------------------------
bool GpsData::can_be_converted_to_vel_msg_(const RMCFrame & rmc_frame)
{
  return rmc_frame.speedOverGroundInMeterPerSecond &&
         rmc_frame.trackAngleTrue;
}

//-----------------------------------------------------------------------------
void GpsData::process_gga_frame_(
  const rclcpp::Time & stamp,
  const std::string & nmea_sentence)
{
  std::cout << " process_gga_frame_ " << std::endl;
  GGAFrame gga_frame(nmea_sentence);
  diagnostics_->updateGGARate(to_romea_duration(stamp));
  if (can_be_converted_to_fix_msg_(gga_frame)) {
    romea::LambertConverter::SecantProjectionParameters parameters;
    parameters.longitude0 = 3.0 / 180. * M_PI;
    parameters.latitude0 = 46.0 / 180. * M_PI;
    parameters.latitude1 = 45.25 / 180. * M_PI;
    parameters.latitude2 = 46.75 / 180. * M_PI;
    parameters.x0 = 1700000;
    parameters.y0 = 5200000;

    LambertConverter converter(parameters, EarthEllipsoid::GRS80);
    WGS84Coordinates wgs84Coordinates;
    wgs84Coordinates.latitude = gga_frame.latitude->toDouble();
    wgs84Coordinates.longitude = gga_frame.longitude->toDouble();
    // std::cout << wgs84Coordinates << std::endl;
    Eigen::Vector2d position = converter.toLambert(wgs84Coordinates);
    std::cout << std::setprecision(10) << " position : " << position.x() << " " << position.y() <<
      std::endl;
    std::cout << " borne voisin distanc e : " << position.x() - 1707862.78 <<
      " " << position.y() - 5166011.93 << std::endl;
    std::cout << " borne edf distance : " << position.x() - 1707865.72 <<
      " " << position.y() - 5165988.47 << std::endl;

    // TODO(Jean) use EURE
    fix_publisher_->publish(stamp, gga_frame);
  }
}

//-----------------------------------------------------------------------------
void GpsData::process_rmc_frame_(
  const rclcpp::Time & stamp,
  const std::string & nmea_sentence)
{
//  std::cout << " process_rmc_frame_ " << std::endl;

  RMCFrame rmc_frame(nmea_sentence);
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
//  std::cout << " process_gsv_frame_ " << std::endl;

  GSVFrame gsv_frame(nmea_sentence);
  if (gsv_frame.sentenceNumber == gsv_frame.numberOfSentences) {
    diagnostics_->updateGSVRate(to_romea_duration(stamp));
  }
}

//-----------------------------------------------------------------------------
void GpsData::process_nmea_sentence(const std::string & nmea_sentence)
{
//  std::cout << nmea_sentence<<std::endl;

  rclcpp::Time stamp = clock_->now();
  nmea_sentence_publisher_->publish(stamp, nmea_sentence);

  switch (NMEAParsing::extractSentenceId(nmea_sentence)) {
    case NMEAParsing::SentenceID::GGA:
      process_gga_frame_(stamp, nmea_sentence);
      break;
    case NMEAParsing::SentenceID::RMC:
      process_rmc_frame_(stamp, nmea_sentence);
      break;
    case NMEAParsing::SentenceID::GSV:
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

}  // namespace romea
