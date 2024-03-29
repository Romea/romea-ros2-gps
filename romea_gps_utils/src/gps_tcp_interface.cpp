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
#include <vector>

// romea core
#include "romea_core_gps/nmea/NMEAParsing.hpp"

// romea ros
#include "romea_common_utils/params/sensor_parameters.hpp"

// local
#include "romea_gps_utils/gps_tcp_interface.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
GpsTcpInterface::GpsTcpInterface(std::shared_ptr<rclcpp::Node> node)
: client_{3000},
  rtcm_client_{3000}
{
  declare_ip(node);
  declare_parameter<int>(node, "nmea_port");
  declare_parameter<int>(node, "rtcm_port");

  auto const & logger = node->get_logger();
  std::string addr = get_ip(node);
  int nmea_port = get_parameter<int>(node, "nmea_port");
  int rtcm_port = get_parameter<int>(node, "rtcm_port");

  client_.connect(addr, nmea_port);
  RCLCPP_INFO(logger, "GNSS NMEA: Connected to %s:%d", addr.c_str(), nmea_port);

  rtcm_client_.connect(addr, rtcm_port);
  RCLCPP_INFO(logger, "GNSS RTCM: Connected to %s:%d", addr.c_str(), rtcm_port);
}

//-----------------------------------------------------------------------------
size_t GpsTcpInterface::read(std::string & data)
{
  data = client_.readline();
  return data.size();
}

//-----------------------------------------------------------------------------
size_t GpsTcpInterface::write_rtcm(const std::vector<std::uint8_t> & data) const
{
  return rtcm_client_.send(data);
}

//-----------------------------------------------------------------------------
std::optional<std::string> GpsTcpInterface::read_nmea_sentence()
{
  std::string raw_sentence;

  read(raw_sentence);
  core::NMEAParsing::removeCRLF(raw_sentence);
  if (core::NMEAParsing::isNMEASentence(raw_sentence)) {
    return raw_sentence;
  } else {
    return std::nullopt;
  }
}

}  // namespace ros2
}  // namespace romea
