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
#include "romea_common_utils/qos.hpp"

// local
#include "romea_gps_driver/gps_tcp_driver.hpp"

namespace romea::ros2
{

//-----------------------------------------------------------------------------
GpsTcpDriver::GpsTcpDriver(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("gps_serial_driver", options)),
  gps_interface_(node_),
  gps_data_(node_),
  rtcm_sub_(nullptr),
  thread_(&GpsTcpDriver::thread_callback, this)
{
  auto callback = std::bind(&GpsTcpDriver::rtcm_callback_, this, std::placeholders::_1);

  rtcm_sub_ =
    node_->create_subscription<mavros_msgs::msg::RTCM>("ntrip/rtcm", reliable(30), callback);
}

//-----------------------------------------------------------------------------
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr GpsTcpDriver::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

//-----------------------------------------------------------------------------
void GpsTcpDriver::rtcm_callback_(mavros_msgs::msg::RTCM::SharedPtr msg)
{
  // RCLCPP_INFO(node_->get_logger(), "RTCM data received (size = %luB)", msg->data.size());
  gps_interface_.write_rtcm(msg->data);
}

//-----------------------------------------------------------------------------
void GpsTcpDriver::thread_callback()
{
  while (rclcpp::ok()) {
    auto nmea_sentence = gps_interface_.read_nmea_sentence();
    // RCLCPP_INFO(node_->get_logger(), "NMEA sentence received");
    if (nmea_sentence.has_value()) {
      gps_data_.process_nmea_sentence(nmea_sentence.value());
    }
  }
}

}  // namespace romea::ros2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ros2::GpsTcpDriver)
