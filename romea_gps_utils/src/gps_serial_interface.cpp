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

// romea core
#include "romea_core_gps/nmea/NMEAParsing.hpp"

// romea ros
#include "romea_common_utils/params/sensor_parameters.hpp"

// local
#include "romea_gps_utils/gps_serial_interface.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
GpsSerialInterface::GpsSerialInterface(std::shared_ptr<rclcpp::Node> node)
{
  declare_device(node);
  declare_baudrate(node);

  try {
    serial_.setPort(get_device(node));
    serial_.setBaudrate(get_baudrate(node));
    serial::Timeout t = serial::Timeout::simpleTimeout(40);
    serial_.setTimeout(t);
    serial_.open();
  } catch (...) {
    throw(std::runtime_error("Unable to connect to device " + get_device(node) ));
  }
}

//-----------------------------------------------------------------------------
size_t GpsSerialInterface::read(std::string & data)
{
  return serial_.readline(data, 65536, "\n");
}

//-----------------------------------------------------------------------------
size_t GpsSerialInterface::write(const std::string & data)
{
  return serial_.write(data);
}

//-----------------------------------------------------------------------------
std::optional<std::string> GpsSerialInterface::read_nmea_sentence()
{
  std::string raw_sentence;

  read(raw_sentence);
  NMEAParsing::removeCRLF(raw_sentence);
  if (NMEAParsing::isNMEASentence(raw_sentence)) {
    return raw_sentence;
  } else {
    return std::nullopt;
  }
}

}  // namespace romea
