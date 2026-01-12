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
#include <unordered_map>
#include <memory>


// gz
#include "gz/common/Profiler.hh"
#include "gz/common/Console.hh"
#include "gz/math/Angle.hh"
#include "gz/math/Vector3.hh"
#include "gz/msgs/Utility.hh"
#include "gz/msgs/stringmsg.pb.h"
#include "gz/msgs/stringmsg_v.pb.h"
#include "gz/msgs/navsat.pb.h"
#include "gz/sensors/Noise.hh"
#include "gz/sensors/SensorFactory.hh"
#include "gz/sensors/SensorTypes.hh"
#include "gz/transport/Node.hh"

// romea
#include "romea_core_gps/nmea/GGAFrame.hpp"
#include "romea_core_gps/nmea/RMCFrame.hpp"
#include "romea_core_gps/nmea/HDTFrame.hpp"
#include "romea_core_common/math/EulerAngles.hpp"
#include "romea_gps_gazebo/nmea_gps_sensor.hpp"


#define gzerr2 (::gz::common::Console::err(__FILE__, __LINE__))
#define gzwarn2 (::gz::common::Console::warn(__FILE__, __LINE__))

namespace
{
const double DEFAULT_DHOP = 1;
const uint8_t DEFAUlT_FIX_STATUS = 0;
const uint16_t DEFAULT_SERVICE = 15;
std::chrono::milliseconds GGA_STAMP_OFFSET(0);
std::chrono::milliseconds RMC_STAMP_OFFSET(0);
std::chrono::milliseconds HDT_STAMP_OFFSET(0);
}  // namespace


namespace romea{
namespace gz{

class NmeaGpsSensorPrivate
{
  public:
    ::gz::transport::Node node;
    ::gz::transport::Node::Publisher pub;
    bool loaded = false;
    ::gz::math::Angle latitude;
    ::gz::math::Angle longitude;
    double altitude = 0.0;
    ::gz::math::Angle yaw;
    ::gz::math::Vector3d velocity;
    bool dual_antenna = false;
    std::unordered_map<::gz::sensors::SensorNoiseType, ::gz::sensors::NoisePtr> noises;
};

//////////////////////////////////////////////////
NmeaGpsSensor::NmeaGpsSensor()
  : dataPtr(std::make_unique<NmeaGpsSensorPrivate>())
{
}

//////////////////////////////////////////////////
NmeaGpsSensor::~NmeaGpsSensor() = default;

//////////////////////////////////////////////////
bool NmeaGpsSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool NmeaGpsSensor::Load(const sdf::Sensor &_sdf)
{
  if (!::gz::sensors::Sensor::Load(_sdf))
    return false;

  if (!_sdf.Element()->HasElement("ns0:gps") && !_sdf.Element()->HasElement("gz:gps"))
  {
    gzerr2 << "No custom configuration for [" << this->Topic() << "]"
           << std::endl;
    return false;
  }


  if(_sdf.Element()->HasElement("dual_antenna"))
  {
    this->dataPtr->dual_antenna = _sdf.Element()->Get<bool>("dual_antenna");
  }

  // auto type = ::gz::sensors::customType(_sdf);

  // gzerr2 << " type" << type << std::endl;
  // if (type != "gps")
  // {
  //   gzerr2 << "Attempting to a load an [gps] sensor, but got type "
  //          << type << "] instead." << std::endl;
  //   return false;
  // }


  // if (_sdf.NmeaGpsSensor() == nullptr)
  // {
  //   gzerr << "Attempting to a load an [nmea_gps] sensor, but received "
  //     << "a null sensor." << std::endl;
  //   return false;
  // }

  if (this->Topic().empty())
    this->SetTopic("/nmea");

  // this->dataPtr->pub =
  //   this->dataPtr->node.Advertise<::gz::custom_msgs::NmeaSentence>(this->Topic());

  this->dataPtr->pub =
    this->dataPtr->node.Advertise<::gz::msgs::StringMsg>(this->Topic());


  if (!this->dataPtr->pub)
  {
    gzerr2 << "Unable to create publisher on topic [" << this->Topic()
           << "]." << std::endl;
    return false;
  }

  // // Load the noise parameters
  // if (_sdf.NmeaGpsSensor()->HorizontalPositionNoise().Type()
  //     != sdf::NoiseType::NONE)
  // {
  //   this->dataPtr->noises[NAVSAT_HORIZONTAL_POSITION_NOISE] =
  //     NoiseFactory::NewNoiseModel(
  //       _sdf.NmeaGpsSensor()->HorizontalPositionNoise());
  // }
  // if (_sdf.NmeaGpsSensor()->VerticalPositionNoise().Type()
  //     != sdf::NoiseType::NONE)
  // {
  //   this->dataPtr->noises[NAVSAT_VERTICAL_POSITION_NOISE] =
  //     NoiseFactory::NewNoiseModel(
  //       _sdf.NmeaGpsSensor()->VerticalPositionNoise());
  // }
  // if (_sdf.NmeaGpsSensor()->HorizontalVelocityNoise().Type()
  //     != sdf::NoiseType::NONE)
  // {
  //   this->dataPtr->noises[NAVSAT_HORIZONTAL_VELOCITY_NOISE] =
  //     NoiseFactory::NewNoiseModel(
  //       _sdf.NmeaGpsSensor()->HorizontalVelocityNoise());
  // }
  // if (_sdf.NmeaGpsSensor()->VerticalVelocityNoise().Type()
  //     != sdf::NoiseType::NONE)
  // {
  //   this->dataPtr->noises[NAVSAT_VERTICAL_VELOCITY_NOISE] =
  //     NoiseFactory::NewNoiseModel(
  //       _sdf.NmeaGpsSensor()->VerticalVelocityNoise());
  // }

  this->dataPtr->loaded = true;
  return true;
}

//////////////////////////////////////////////////
bool NmeaGpsSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool NmeaGpsSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  GZ_PROFILE("NmeaGpsSensor::Update");
  if (!this->dataPtr->loaded)
  {
    gzerr2 << "Not loaded, update ignored.\n";
    return false;
  }

  // // Apply noise
  // auto iter = this->dataPtr->noises.find(NAVSAT_HORIZONTAL_POSITION_NOISE);
  // if (iter != this->dataPtr->noises.end())
  // {
  //   this->SetLatitude(GZ_DTOR(iter->second->Apply(this->Latitude().Degree())));
  //   this->SetLongitude(GZ_DTOR(iter->second->Apply(
  //       this->Longitude().Degree())));
  // }
  // iter = this->dataPtr->noises.find(NAVSAT_VERTICAL_POSITION_NOISE);
  // if (iter != this->dataPtr->noises.end())
  // {
  //   this->SetAltitude(iter->second->Apply(this->Altitude()));
  // }
  // iter = this->dataPtr->noises.find(NAVSAT_HORIZONTAL_VELOCITY_NOISE);
  // if (iter != this->dataPtr->noises.end())
  // {
  //   this->dataPtr->velocity.X(iter->second->Apply(this->dataPtr->velocity.X()));
  //   this->dataPtr->velocity.Y(iter->second->Apply(this->dataPtr->velocity.Y()));
  // }
  // iter = this->dataPtr->noises.find(NAVSAT_VERTICAL_VELOCITY_NOISE);
  // if (iter != this->dataPtr->noises.end())
  // {
  //   this->dataPtr->velocity.Z(iter->second->Apply(this->dataPtr->velocity.Z()));
  // }

  // // normalise so that it is within +/- 180
  // this->dataPtr->latitude.Normalize();
  // this->dataPtr->longitude.Normalize();

  double latitude = this->dataPtr->latitude.Radian();
  double longitude = this->dataPtr->longitude.Radian();
  double altitude = this->dataPtr->altitude;
  double v_east = this->dataPtr->velocity.X();
  double v_north = this->dataPtr->velocity.Y();
  // double v_up = this->dataPtr->velocity.Z();
  double yaw = this->dataPtr->yaw.Radian();

  auto gga_stamp = _now + GGA_STAMP_OFFSET;

  romea::core::GGAFrame gga_frame;
  gga_frame.fixTime = romea::core::FixTime(
    gga_stamp.count()/1000000000, gga_stamp.count()%1000000000);
  gga_frame.talkerId = romea::core::TalkerId::GP;
  gga_frame.latitude = romea::core::Latitude(latitude);
  gga_frame.longitude = romea::core::Longitude(longitude);
  gga_frame.altitudeAboveGeoid = altitude;
  gga_frame.fixQuality = romea::core::FixQuality::SIMULATION_FIX;
  gga_frame.geoidHeight = 0;
  gga_frame.horizontalDilutionOfPrecision = DEFAULT_DHOP;
  gga_frame.numberSatellitesUsedToComputeFix = 0;
  publishNmeaSentence(gga_stamp, gga_frame.toNMEA());

  auto rmc_stamp = _now + RMC_STAMP_OFFSET;

  romea::core::RMCFrame rmc_frame;
  rmc_frame.fixTime = core::FixTime(
    rmc_stamp.count()/1000000000, rmc_stamp.count()%1000000000);
  rmc_frame.status = core::RMCFrame::Status::Void;
  rmc_frame.talkerId = core::TalkerId::GP;
  rmc_frame.latitude = romea::core::Latitude(latitude);
  rmc_frame.longitude = romea::core::Longitude(longitude);
  rmc_frame.trackAngleTrue = M_PI_2 - std::atan2(v_north, v_east);
  rmc_frame.speedOverGroundInMeterPerSecond = std::sqrt(v_east * v_east + v_north * v_north);
  rmc_frame.fixQuality = core::FixQuality::SIMULATION_FIX;
  publishNmeaSentence(rmc_stamp, rmc_frame.toNMEA());

  if(this->dataPtr->dual_antenna)
  {
    auto hdt_stamp = _now + HDT_STAMP_OFFSET;

    romea::core::HDTFrame hdt_frame;
    hdt_frame.talkerId = core::TalkerId::GP;
    hdt_frame.heading = romea::core::between0And2Pi(M_PI_2 - yaw);
    hdt_frame.trueNorth = true;
    publishNmeaSentence(hdt_stamp, hdt_frame.toNMEA());
  }

  return true;
}

//////////////////////////////////////////////////
void NmeaGpsSensor::SetLatitude(const ::gz::math::Angle &_latitude)
{
  this->dataPtr->latitude = _latitude;
}

//////////////////////////////////////////////////
const ::gz::math::Angle &NmeaGpsSensor::Latitude() const
{
  return this->dataPtr->latitude;
}

//////////////////////////////////////////////////
void NmeaGpsSensor::SetAltitude(double _altitude)
{
  this->dataPtr->altitude = _altitude;
}

//////////////////////////////////////////////////
double NmeaGpsSensor::Altitude() const
{
  return this->dataPtr->altitude;
}

//////////////////////////////////////////////////
void NmeaGpsSensor::SetLongitude(const ::gz::math::Angle &_longitude)
{
  this->dataPtr->longitude = _longitude;
}

//////////////////////////////////////////////////
const ::gz::math::Angle &NmeaGpsSensor::Longitude() const
{
  return this->dataPtr->longitude;
}

//////////////////////////////////////////////////
void NmeaGpsSensor::SetYaw(const ::gz::math::Angle & _yaw)
{
  this->dataPtr->yaw = _yaw;
}

//////////////////////////////////////////////////
const ::gz::math::Angle &NmeaGpsSensor::Yaw() const
{
  return this->dataPtr->yaw;
}

//////////////////////////////////////////////////
void NmeaGpsSensor::SetVelocity(const ::gz::math::Vector3d &_vel)
{
  this->dataPtr->velocity = _vel;
}

//////////////////////////////////////////////////
const ::gz::math::Vector3d &NmeaGpsSensor::Velocity() const
{
  return this->dataPtr->velocity;
}

//////////////////////////////////////////////////
void NmeaGpsSensor::SetPosition(
  const ::gz::math::Angle &_latitude,
  const ::gz::math::Angle &_longitude,
  double _altitude)
{
  this->SetLatitude(_latitude);
  this->SetLongitude(_longitude);
  this->SetAltitude(_altitude);
}

//////////////////////////////////////////////////
bool NmeaGpsSensor::HasConnections() const
{
  return this->dataPtr->pub && this->dataPtr->pub.HasConnections();
}

//////////////////////////////////////////////////
void NmeaGpsSensor::publishNmeaSentence(
  const std::chrono::steady_clock::duration & stamp,
  const std::string nmea_sentence)
{
  ::gz::msgs::StringMsg msg;
  *msg.mutable_header()->mutable_stamp() = ::gz::msgs::Convert(stamp);
  msg.set_data(nmea_sentence);

  auto frame_id_data = msg.mutable_header()->add_data();
  frame_id_data->set_key("frame_id");
  frame_id_data->add_value(this->FrameId());

  this->AddSequence(msg.mutable_header());
  // std::cout << msg.data() << std::endl;
  this->dataPtr->pub.Publish(msg);
}


}  // namespace gz
}  // namespace romea


