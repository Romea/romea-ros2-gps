// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// romea ros
#include <romea_common_utils/params/node_parameters.hpp>
#include <romea_common_utils/params/eigen_parameters.hpp>

// local
#include "romea_gps_utils/gps_parameters.hpp"

namespace
{
const char gps_fix_uere_param_name[] = "gps.gps_fix_uere";
const char dgps_fix_uere_param_name[] = "gps.dgps_fix_uere";
const char float_rtk_fix_uere_param_name[] = "gps.float_rtk_fix_uere";
const char rtk_fix_uere_param_name[] = "gps.rtk_fix_uere";
const char simulation_fix_uere_param_name[] = "gps.simulation_fix_uere";
}

namespace romea
{

//-----------------------------------------------------------------------------
void declare_gps_gps_fix_eure(rclcpp::Node::SharedPtr node)
{
  declare_parameter<double>(node, gps_fix_uere_param_name);
}

//-----------------------------------------------------------------------------
void declare_gps_dgps_fix_eure(rclcpp::Node::SharedPtr node)
{
  declare_parameter<double>(node, dgps_fix_uere_param_name);
}

//-----------------------------------------------------------------------------
void declare_gps_float_rtk_fix_eure(rclcpp::Node::SharedPtr node)
{
  declare_parameter<double>(node, float_rtk_fix_uere_param_name);
}

//-----------------------------------------------------------------------------
void declare_gps_rtk_fix_eure(rclcpp::Node::SharedPtr node)
{
  declare_parameter<double>(node, rtk_fix_uere_param_name);
}

//-----------------------------------------------------------------------------
void declare_gps_simulation_fix_eure(rclcpp::Node::SharedPtr node)
{
  declare_parameter<double>(node, simulation_fix_uere_param_name);
}

//-----------------------------------------------------------------------------
void declare_gps_antenna_body_position(rclcpp::Node::SharedPtr node)
{
  return declare_eigen_xyz_vector_parameter<Eigen::Vector3d>(node, "gps");
}

//-----------------------------------------------------------------------------
double get_gps_gps_fix_eure(rclcpp::Node::SharedPtr node)
{
  return get_parameter<double>(node, gps_fix_uere_param_name);
}


//-----------------------------------------------------------------------------
double get_gps_dgps_fix_eure(rclcpp::Node::SharedPtr node)
{
  return get_parameter<double>(node, dgps_fix_uere_param_name);
}

//-----------------------------------------------------------------------------
double get_gps_float_rtk_fix_eure(rclcpp::Node::SharedPtr node)
{
  return get_parameter<double>(node, float_rtk_fix_uere_param_name);
}

//-----------------------------------------------------------------------------
double get_gps_rtk_fix_eure(rclcpp::Node::SharedPtr node)
{
  return get_parameter<double>(node, rtk_fix_uere_param_name);
}

//-----------------------------------------------------------------------------
double get_gps_simulation_fix_eure(rclcpp::Node::SharedPtr node)
{
  return get_parameter<double>(node, simulation_fix_uere_param_name);
}

//-----------------------------------------------------------------------------
Eigen::Vector3d get_gps_antenna_body_position(rclcpp::Node::SharedPtr node)
{
  return get_eigen_xyz_vector_parameter<Eigen::Vector3d>(node, "gps");
}


}  // namespace romea
