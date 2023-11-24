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

#ifndef ROMEA_GPS_UTILS__GPS_SERIAL_INTERFACE_HPP_
#define ROMEA_GPS_UTILS__GPS_SERIAL_INTERFACE_HPP_

// std
#include <optional>
#include <memory>
#include <string>

// ros
#include "rclcpp/rclcpp.hpp"

// serial
#include "serial/serial.h"


namespace romea
{
namespace ros2
{

class GpsSerialInterface
{
public:
  explicit GpsSerialInterface(std::shared_ptr<rclcpp::Node> node);

  size_t read(std::string & data);

  size_t write(const std::string & data);

  std::optional<std::string> read_nmea_sentence();

protected:
  serial::Serial serial_;
  std::string device_;
  int baudrate_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_GPS_UTILS__GPS_SERIAL_INTERFACE_HPP_
