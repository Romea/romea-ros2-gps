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

#ifndef ROMEA_GPS_GAZEBO__GAZEBO_ROS_GPS_SENSOR_HPP_
#define ROMEA_GPS_GAZEBO__GAZEBO_ROS_GPS_SENSOR_HPP_

// std
#include <memory>

// ros
#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/GpsSensor.hh"
#include "gazebo/common/Events.hh"

namespace romea
{
namespace ros2
{

class GazeboRosGpsSensorPrivate;

/// Plugin to attach to a gazebo GPS sensor and publish ROS message of output
/**
  Example Usage:
  \code{.xml}
    <sensor name="my_gps" type="gps">
      <!-- ensure the sensor is active (required) -->
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <plugin name="my_gps_plugin" filename="libgazebo_ros_gps_sensor.so">
        <ros>
          <!-- publish to /gps/data -->
          <namespace>/gps</namespace>
          <remapping>~/out:=data</remapping>
        </ros>
      </plugin>
    </sensor>
  \endcode
*/

class GazeboRosGpsSensor : public gazebo::SensorPlugin
{
public:
  /// Constructor.
  GazeboRosGpsSensor();

  /// Destructor.
  virtual ~GazeboRosGpsSensor();

  // Documentation Inherited
  void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosGpsSensorPrivate> impl_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_GPS_GAZEBO__GAZEBO_ROS_GPS_SENSOR_HPP_
