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

#ifndef ROMEA_GPS_GAZEBO__NMEA_GPS_SENSOR_HPP_
#define ROMEA_GPS_GAZEBO__NMEA_GPS_SENSOR_HPP_

// std
#include <memory>
#include <string>

// gz
#include "gz/custom_msgs/nmea_sentence.pb.h"
#include "gz/sensors/Sensor.hh"
#include "gz/sensors/Util.hh"

// sdf
#include "sdf/Sensor.hh"

namespace romea
{
namespace gz
{

class NmeaGpsSensorPrivate;

class NmeaGpsSensor : public ::gz::sensors::Sensor
{
public:
  NmeaGpsSensor();
  virtual ~NmeaGpsSensor();

  bool Load(const sdf::Sensor & _sdf) override;
  bool Load(sdf::ElementPtr _sdf) override;
  bool Init() override;
  bool Update(const std::chrono::steady_clock::duration & _now) override;
  bool HasConnections() const override;

  void SetLatitude(const ::gz::math::Angle & _latitude);
  const ::gz::math::Angle & Latitude() const;

  void SetLongitude(const ::gz::math::Angle & _longitude);
  const ::gz::math::Angle & Longitude() const;

  void SetAltitude(double _altitude);
  double Altitude() const;

  void SetYaw(const ::gz::math::Angle & _yaw);
  const ::gz::math::Angle & Yaw() const;

  void SetPosition(
    const ::gz::math::Angle & _latitude,
    const ::gz::math::Angle & _longitude,
    double _altitude = 0.0);

  void SetVelocity(const ::gz::math::Vector3d & _vel);
  const ::gz::math::Vector3d & Velocity() const;

private:
  void publishNmeaSentence(
    const std::chrono::steady_clock::duration & stamp, const std::string nmea_sentence);

private:
  std::unique_ptr<NmeaGpsSensorPrivate> dataPtr;
};

}  // namespace gz
}  // namespace romea

#endif  // ROMEA_GPS_GAZEBO__NMEA_GPS_SENSOR_HPP_
