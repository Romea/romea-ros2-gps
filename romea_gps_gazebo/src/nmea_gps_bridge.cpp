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

// romea
#include "romea_gps_gazebo/nmea_gps_bridge.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
NmeaGpsGzBridge::NmeaGpsGzBridge(const rclcpp::NodeOptions & options)
: Node("nmea_gps_bridge", options)
{
  this->declare_parameter<std::string>("ros_topic_name");
  this->declare_parameter<std::string>("gz_topic_name");
  this->get_parameter<std::string>("ros_topic_name", ros_topic_name_);
  this->get_parameter<std::string>("gz_topic_name", gz_topic_name_);

  RCLCPP_INFO(this->get_logger(), "nmea_gps_bridge");
  rclcpp::PublisherOptions pub_options;
  pub_options.event_callbacks.matched_callback = [this](rclcpp::MatchedInfo & s) {
    if (s.current_count > 0 && !gz_subscribed_) {
      // RCLCPP_INFO(this->get_logger(), "Subscribing to Gazebo /romea/gps/nmea...");
      gz_node_.Subscribe(this->gz_topic_name_, &NmeaGpsGzBridge::gz_callback_, this);
      gz_subscribed_ = true;
    } else if (s.current_count == 0 && gz_subscribed_) {
      // RCLCPP_INFO(this->get_logger(), "No ROS subscribers, unsubscribing from Gazebo");
      gz_node_.Unsubscribe(this->gz_topic_name_);
      gz_subscribed_ = false;
    }
  };

  pub_ = this->create_publisher<nmea_msgs::msg::Sentence>(
    ros_topic_name_, rclcpp::SensorDataQoS().reliable(), pub_options);
}

//-----------------------------------------------------------------------------
void NmeaGpsGzBridge::gz_callback_(const gz::msgs::StringMsg & msg)
{
  const auto & gz_stamp = msg.header().stamp();
  if (pub_->get_subscription_count() == 0) return;

  nmea_msgs::msg::Sentence nmea_msg;
  nmea_msg.header.stamp = rclcpp::Time(gz_stamp.sec(), gz_stamp.nsec(), RCL_ROS_TIME);
  nmea_msg.header.frame_id = msg.header().data(0).value(0);
  nmea_msg.sentence = msg.data();

  pub_->publish(nmea_msg);
}

}  // namespace ros2
}  // namespace romea

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ros2::NmeaGpsGzBridge)
