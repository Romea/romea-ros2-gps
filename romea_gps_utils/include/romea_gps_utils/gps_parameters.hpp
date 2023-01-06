// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license


#ifndef ROMEA_GPS_UTILS__GPS_PARAMETERS_HPP_
#define ROMEA_GPS_UTILS__GPS_PARAMETERS_HPP_

// eigen
#include <Eigen/Geometry>

// ros
#include <rclcpp/node.hpp>


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
