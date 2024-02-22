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

// romea ros
#include <romea_common_utils/qos.hpp>

// local
#include "romea_gps_driver/gps_serial_driver.hpp"


namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
GpsSerialDriver::GpsSerialDriver(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("gps_serial_driver", options)),
  gps_interface_(node_),
  gps_data_(node_),
  rtcm_sub_(nullptr),
  thread_(&GpsSerialDriver::thread_callback, this)
{
  auto callback = std::bind(&GpsSerialDriver::rtcm_callback_, this, std::placeholders::_1);

  rtcm_sub_ = node_->create_subscription<mavros_msgs::msg::RTCM>(
    "ntrip/rtcm", best_effort(1), callback);
}

//-----------------------------------------------------------------------------
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
GpsSerialDriver::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

//-----------------------------------------------------------------------------
void GpsSerialDriver::rtcm_callback_(mavros_msgs::msg::RTCM::SharedPtr msg)
{
  std::string data(msg->data.begin(), msg->data.end());
  gps_interface_.write(data);
}

//-----------------------------------------------------------------------------
void GpsSerialDriver::thread_callback()
{
  while (rclcpp::ok()) {
    auto nmea_sentence = gps_interface_.read_nmea_sentence();
    if (nmea_sentence.has_value()) {
      gps_data_.process_nmea_sentence(nmea_sentence.value());
    }
  }
}

}  // namespace ros2
}  // namespace romea

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ros2::GpsSerialDriver)
