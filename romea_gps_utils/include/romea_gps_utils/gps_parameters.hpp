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



#ifndef ROMEA_GPS_UTILS__GPS_PARAMETERS_HPP_
#define ROMEA_GPS_UTILS__GPS_PARAMETERS_HPP_

// eigen
#include <Eigen/Geometry>

// ros
#include "rclcpp/node.hpp"


namespace romea
{

void declare_gps_gps_fix_eure(rclcpp::Node::SharedPtr node);
void declare_gps_dgps_fix_eure(rclcpp::Node::SharedPtr node);
void declare_gps_float_rtk_fix_eure(rclcpp::Node::SharedPtr node);
void declare_gps_rtk_fix_eure(rclcpp::Node::SharedPtr node);
void declare_gps_simulation_fix_eure(rclcpp::Node::SharedPtr node);
void declare_gps_antenna_body_position(rclcpp::Node::SharedPtr node);

double get_gps_gps_fix_eure(rclcpp::Node::SharedPtr node);
double get_gps_dgps_fix_eure(rclcpp::Node::SharedPtr node);
double get_gps_float_rtk_fix_eure(rclcpp::Node::SharedPtr node);
double get_gps_rtk_fix_eure(rclcpp::Node::SharedPtr node);
double get_gps_simulation_fix_eure(rclcpp::Node::SharedPtr node);
Eigen::Vector3d get_gps_antenna_body_position(rclcpp::Node::SharedPtr node);


}  // namespace romea

#endif  // ROMEA_GPS_UTILS__GPS_PARAMETERS_HPP_
