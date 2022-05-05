// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include "romea_gps_gazebo/gazebo_ros_gps_sensor.hpp"
#include <romea_core_gps/nmea/GGAFrame.hpp>
#include <romea_core_gps/nmea/RMCFrame.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nmea_msgs/msg/sentence.hpp>

#include <iostream>
#include <memory>
#include <string>

namespace  {

const double DEFAULT_DHOP=1;
const unsigned char DEFAUlT_FIX_STATUS=0;
const unsigned short DEFAULT_SERVICE= 15;
const rclcpp::Time GGA_STAMP_OFFSET= rclcpp::Time(0,0);
const rclcpp::Time RMC_STAMP_OFFSET= rclcpp::Time(0,5000000);
}

namespace romea
{

class GazeboRosGpsSensorPrivate
{
public:
  /// Node for ros communication
  gazebo_ros::Node::SharedPtr ros_node_;
  /// Publish for gps message
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
  rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr nmea_sentence_pub_;

  /// GPS message modified each update
  sensor_msgs::msg::NavSatFix::SharedPtr fix_msg_;
  geometry_msgs::msg::TwistStamped::SharedPtr vel_msg_;
  nmea_msgs::msg::Sentence::SharedPtr nmea_gga_sentence_msg_;
  nmea_msgs::msg::Sentence::SharedPtr nmea_rmc_sentence_msg_;

  /// GPS sensor this plugin is attached to
  gazebo::sensors::GpsSensorPtr sensor_;
  /// Event triggered when sensor updates
  gazebo::event::ConnectionPtr sensor_update_event_;

  /// Publish latest gps data to ROS
  void OnUpdate();
};

GazeboRosGpsSensor::GazeboRosGpsSensor()
  : impl_(std::make_unique<GazeboRosGpsSensorPrivate>())
{
}

GazeboRosGpsSensor::~GazeboRosGpsSensor()
{
}

void GazeboRosGpsSensor::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  //  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf, _sensor);
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  impl_->sensor_ = std::dynamic_pointer_cast<gazebo::sensors::GpsSensor>(_sensor);
  if (!impl_->sensor_) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Parent is not a GPS sensor. Exiting.");
    return;
  }

  unsigned int fix_status = DEFAUlT_FIX_STATUS;
  if (_sdf->HasElement("status"))
  {
    _sdf->GetElement("status")->GetValue()->Get(fix_status);
  }

  unsigned int service = DEFAULT_SERVICE;
  if (_sdf->HasElement("service"))
  {
    _sdf->GetElement("service")->GetValue()->Get(service);
  }

  //  Init fix publisher
  impl_->fix_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::NavSatFix>(
        "~/fix", qos.get_publisher_qos("~/fix", rclcpp::SensorDataQoS().reliable()));

  // Init fix msg
  impl_->fix_msg_ = std::make_shared<sensor_msgs::msg::NavSatFix>();
  impl_->fix_msg_->header.frame_id = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  using SNT = gazebo::sensors::SensorNoiseType;
  impl_->fix_msg_->position_covariance[0] =
      gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::GPS_POSITION_LATITUDE_NOISE_METERS));
  impl_->fix_msg_->position_covariance[4] =
      gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::GPS_POSITION_LONGITUDE_NOISE_METERS));
  impl_->fix_msg_->position_covariance[8] =
      gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::GPS_POSITION_ALTITUDE_NOISE_METERS));
  impl_->fix_msg_->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  impl_->fix_msg_->status.status = static_cast<sensor_msgs::msg::NavSatStatus::_status_type>(fix_status);
  impl_->fix_msg_->status.service = static_cast<sensor_msgs::msg::NavSatStatus::_service_type>(service);


  //  Init vel publisher
  impl_->vel_pub_ = impl_->ros_node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        "~/vel", qos.get_publisher_qos("~/vel", rclcpp::SensorDataQoS().reliable()));

  // Init vel message
  impl_->vel_msg_= std::make_shared<geometry_msgs::msg::TwistStamped>();
  impl_->vel_msg_->header.frame_id = gazebo_ros::SensorFrameID(*_sensor, *_sdf);


  //  Init nmea sentence publisher
  impl_->nmea_sentence_pub_ = impl_->ros_node_->create_publisher<nmea_msgs::msg::Sentence>(
        "~/nmea_sentence", qos.get_publisher_qos("~/nmea_sentence", rclcpp::SensorDataQoS().reliable()));

  // Init nmea messages
  impl_->nmea_gga_sentence_msg_= std::make_shared<nmea_msgs::msg::Sentence>();
  impl_->nmea_gga_sentence_msg_->header.frame_id = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  impl_->nmea_rmc_sentence_msg_= std::make_shared<nmea_msgs::msg::Sentence>();
  impl_->nmea_rmc_sentence_msg_->header.frame_id = gazebo_ros::SensorFrameID(*_sensor, *_sdf);


  impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
        std::bind(&GazeboRosGpsSensorPrivate::OnUpdate, impl_.get()));
}

void GazeboRosGpsSensorPrivate::OnUpdate()
{
  //  #ifdef IGN_PROFILER_ENABLE
  //  IGN_PROFILE("GazeboRosGpsSensorPrivate::OnUpdate");
  //  IGN_PROFILE_BEGIN("fill ROS message");
  //  #endif

  auto stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
        sensor_->LastUpdateTime());

  double latitude  = sensor_->Latitude().Degree();
  double longitude = sensor_->Longitude().Degree();
  double altitude  = sensor_->Altitude();
  double v_east = sensor_->VelocityEast();
  double v_north = sensor_->VelocityNorth();
  double v_up = sensor_->VelocityUp();

  romea::GGAFrame gga_frame;
  gga_frame.fixTime =romea::FixTime(stamp.sec+GGA_STAMP_OFFSET.seconds(),
                                    stamp.nanosec+GGA_STAMP_OFFSET.nanoseconds());
  gga_frame.talkerId = romea::TalkerId::GP;
  gga_frame.latitude = latitude/180.*M_PI;
  gga_frame.longitude = longitude/180.*M_PI;
  gga_frame.altitudeAboveGeoid = altitude;
  gga_frame.fixQuality = romea::FixQuality::SIMULATION_FIX;
  gga_frame.geoidHeight = 0;
  gga_frame.horizontalDilutionOfPrecision =DEFAULT_DHOP;
  gga_frame.numberSatellitesUsedToComputeFix = 0;
  nmea_gga_sentence_msg_->sentence = gga_frame.toNMEA();
  nmea_sentence_pub_->publish(*nmea_gga_sentence_msg_);

  romea::RMCFrame rmc_frame;
  rmc_frame.fixTime =romea::FixTime(stamp.sec+RMC_STAMP_OFFSET.seconds(),
                                    stamp.nanosec+RMC_STAMP_OFFSET.nanoseconds());
  rmc_frame.status = romea::RMCFrame::Status::Void;
  rmc_frame.talkerId = romea::TalkerId::GP;
  rmc_frame.latitude = latitude/180.*M_PI;
  rmc_frame.longitude = longitude/180.*M_PI;
  rmc_frame.trackAngleTrue = M_PI_2-std::atan2(v_north,v_east);
  rmc_frame.speedOverGroundInMeterPerSecond = std::sqrt(v_east*v_east + v_north*v_north);
  nmea_rmc_sentence_msg_->sentence = rmc_frame.toNMEA();
  nmea_sentence_pub_->publish(*nmea_rmc_sentence_msg_);


  fix_msg_->header.stamp = stamp ;
  fix_msg_->latitude = latitude;
  fix_msg_->longitude = longitude;
  fix_msg_->altitude = altitude;
  fix_pub_->publish(*fix_msg_);

  vel_msg_->header.stamp = stamp;
  vel_msg_->twist.linear.x = v_east;
  vel_msg_->twist.linear.y = v_north;
  vel_msg_->twist.linear.z = v_up;
  vel_pub_->publish(*vel_msg_);



  //  #ifdef IGN_PROFILER_ENABLE
  //  IGN_PROFILE_END();
  //  IGN_PROFILE_BEGIN("publish");
  //  #endif
  // Publish message
  //  fix_pub_->publish(*fix_msg_);
  //  #ifdef IGN_PROFILER_ENABLE
  //  IGN_PROFILE_END();
  //  #endif
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosGpsSensor)

} // namespace gazebo
