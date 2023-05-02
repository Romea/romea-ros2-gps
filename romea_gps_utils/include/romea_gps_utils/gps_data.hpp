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

#ifndef ROMEA_GPS_UTILS__GPS_DATA_HPP_
#define ROMEA_GPS_UTILS__GPS_DATA_HPP_

// std
#include <memory>
#include <string>

// serial
#include "serial/serial.h"

// romea core
#include "romea_core_gps/nmea/GGAFrame.hpp"
#include "romea_core_gps/nmea/RMCFrame.hpp"
#include "romea_core_gps/nmea/GSVFrame.hpp"

// romea ros
#include "romea_common_utils/publishers/stamped_data_publisher.hpp"
#include "romea_common_utils/publishers/diagnostic_publisher.hpp"

// local
#include "romea_gps_utils/gps_data_diagnostics.hpp"


namespace romea
{

class GpsData
{
public:
  explicit GpsData(std::shared_ptr<rclcpp::Node> node);

  void process_nmea_sentence(const std::string & nmea_sentence);

protected:
  void init_diagnostics_(std::shared_ptr<rclcpp::Node> node);

  void init_publishers_(std::shared_ptr<rclcpp::Node> node);

  void init_timer_(std::shared_ptr<rclcpp::Node> node);

  void process_gga_frame_(
    const rclcpp::Time & stamp,
    const std::string & nmea_sentence);

  void process_rmc_frame_(
    const rclcpp::Time & stamp,
    const std::string & nmea_sentence);

  void process_gsv_frame_(
    const rclcpp::Time & stamp,
    const std::string & nmea_sentence);


  bool can_be_converted_to_fix_msg_(const GGAFrame & gga_frame);

  bool can_be_converted_to_vel_msg_(const RMCFrame & rmc_frame);

  void timer_callback_();

protected:
  std::shared_ptr<rclcpp::Clock> clock_;
  std::shared_ptr<rclcpp::TimerBase> timer_;

  std::unique_ptr<GpsDataDiagnostics> diagnostics_;
  std::shared_ptr<StampedPublisherBase<GGAFrame>> fix_publisher_;
  std::shared_ptr<StampedPublisherBase<RMCFrame>> vel_publisher_;
  std::shared_ptr<StampedPublisherBase<std::string>> nmea_sentence_publisher_;
  std::shared_ptr<StampedPublisherBase<DiagnosticReport>> diagnostics_publisher_;
};

}  // namespace romea

#endif  // ROMEA_GPS_UTILS__GPS_DATA_HPP_
