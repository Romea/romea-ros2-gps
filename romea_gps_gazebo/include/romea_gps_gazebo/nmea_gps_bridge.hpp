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

#ifndef ROMEA_GPS_GAZEBO__NMEA_GPS_BRIDGE_HPP_
#define ROMEA_GPS_GAZEBO__NMEA_GPS_BRIDGE_HPP_

// std
#include <memory>
#include <string>

// gz
#include "gz/transport/Node.hh"
#include "gz/msgs/stringmsg.pb.h"

// ros
#include "rclcpp/rclcpp.hpp"
#include "nmea_msgs/msg/sentence.hpp"


namespace romea
{
namespace ros2
{

class NmeaGpsGzBridge : public rclcpp::Node
{
public:
  explicit NmeaGpsGzBridge(const rclcpp::NodeOptions & options);

private:
  void gz_callback_(const gz::msgs::StringMsg & msg);

private:
  std::string ros_topic_name_;
  rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr pub_;

  std::string gz_topic_name_;
  gz::transport::Node gz_node_;
  bool gz_subscribed_{false};
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_GPS_GAZEBO__NMEA_GPS_BRIDGE_HPP_
