#ifndef _romea_GpsDataConversions_hpp_
#define _romea_GpsDataConversions_hpp_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nmea_msgs/msg/sentence.hpp>

#include <romea_core_gps/nmea/NMEAParsing.hpp>
#include <romea_core_gps/nmea/GGAFrame.hpp>
#include <romea_core_gps/nmea/RMCFrame.hpp>
#include <romea_core_gps/nmea/GSVFrame.hpp>

namespace romea {

void to_ros_msg(const rclcpp::Time & stamp,
                const std::string & frame_id,
                const std::string & raw_sentence,
                nmea_msgs::msg::Sentence & msg);

void to_ros_msg(const rclcpp::Time & stamp,
                const std::string & frame_id,
                const GGAFrame & gga_frame,
                sensor_msgs::msg::NavSatFix & msg);

void to_ros_msg(const rclcpp::Time & stamp,
                const std::string & frame_id,
                const RMCFrame & rmc_frame,
                geometry_msgs::msg::TwistStamped & msg);



}

#endif
