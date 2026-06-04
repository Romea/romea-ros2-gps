/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "romea_gps_gazebo/nmea_gps_system.hpp"

#include <gz/sim/components/CustomSensor.hh>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "gz/common/Console.hh"
#include "gz/common/Profiler.hh"
#include "gz/math/Helpers.hh"
#include "gz/msgs/navsat.pb.h"
#include "gz/msgs/stringmsg_v.pb.h"
#include "gz/plugin/Register.hh"
#include "gz/sensors/SensorFactory.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/transport/Node.hh"
#include "romea_gps_gazebo/nmea_gps_sensor.hpp"
#include "sdf/Sensor.hh"

#define gzerr2 (::gz::common::Console::err(__FILE__, __LINE__))
#define gzwarn2 (::gz::common::Console::warn(__FILE__, __LINE__))

namespace romea
{
namespace gz
{

class NmeaGps::Implementation
{
public:
  // gz::sim::components::CustomSensor

  std::unordered_map<::gz::sim::Entity, std::unique_ptr<NmeaGpsSensor>> entitySensorMap;

  ::gz::sensors::SensorFactory sensorFactory;

  std::unordered_set<::gz::sim::Entity> newSensors;

  bool initialized = false;

  void CreateSensors(const ::gz::sim::EntityComponentManager & _ecm);

  void Update(const ::gz::sim::EntityComponentManager & _ecm);

  void RemoveSensors(const ::gz::sim::EntityComponentManager & _ecm);

  void AddSensor(
    const ::gz::sim::EntityComponentManager & _ecm,
    const ::gz::sim::Entity _entity,
    const ::gz::sim::components::CustomSensor * _custom,
    const ::gz::sim::components::ParentEntity * _parent);
};

//////////////////////////////////////////////////
NmeaGps::NmeaGps() : System(), dataPtr(::gz::utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
void NmeaGps::PreUpdate(
  const ::gz::sim::UpdateInfo & /*_info*/, ::gz::sim::EntityComponentManager & _ecm)
{
  GZ_PROFILE("NmeaGps::PreUpdate");

  // Create components
  for (auto entity : this->dataPtr->newSensors) {
    auto it = this->dataPtr->entitySensorMap.find(entity);
    if (it == this->dataPtr->entitySensorMap.end()) {
      gzerr2 << "Entity [" << entity << "] isn't in sensor map, this shouldn't happen."
             << std::endl;
      continue;
    }
    // Set topic and liner velocity components
    _ecm.CreateComponent(entity, ::gz::sim::components::SensorTopic(it->second->Topic()));
    _ecm.CreateComponent(entity, ::gz::sim::components::WorldLinearVelocity());
  }
  this->dataPtr->newSensors.clear();
}

//////////////////////////////////////////////////
void NmeaGps::PostUpdate(
  const ::gz::sim::UpdateInfo & _info, const ::gz::sim::EntityComponentManager & _ecm)
{
  GZ_PROFILE("NmeaGps::PostUpdate");
  // gzerr2 << "gps system post update "<< std::endl;

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero()) {
    // gzwarn << "Detected jump back in time ["
    //        << std::chrono::duration<double>(_info.dt).count()
    //        << "s]. System may not work properly." << std::endl;
  }

  this->dataPtr->CreateSensors(_ecm);

  // Only update and publish if not paused.
  if (!_info.paused) {
    // check to see if update is necessary
    // we only update if there is at least one sensor that needs data
    // and that sensor has subscribers.
    // note: gz-sensors does its own throttling. Here the check is mainly
    // to avoid doing work in the NmeaGps::Implementation::Update function
    bool needsUpdate = false;
    for (auto & it : this->dataPtr->entitySensorMap) {
      if (it.second->NextDataUpdateTime() <= _info.simTime && it.second->HasConnections()) {
        needsUpdate = true;
        break;
      }
    }
    if (!needsUpdate) return;

    this->dataPtr->Update(_ecm);

    for (auto & it : this->dataPtr->entitySensorMap) {
      it.second.get()->::gz::sensors::Sensor::Update(_info.simTime, false);
    }
  }

  this->dataPtr->RemoveSensors(_ecm);
}

//////////////////////////////////////////////////
void NmeaGps::Implementation::AddSensor(
  const ::gz::sim::EntityComponentManager & _ecm,
  const ::gz::sim::Entity _entity,
  const ::gz::sim::components::CustomSensor * _custom,
  const ::gz::sim::components::ParentEntity * _parent)
{
  // create sensor
  std::string sensorScopedName =
    ::gz::sim::removeParentScope(::gz::sim::scopedName(_entity, _ecm, "::", false), "::");

  sdf::Sensor data = _custom->Data();
  data.SetName(sensorScopedName);

  // check topic
  if (data.Topic().empty()) {
    std::string topic = ::gz::sim::scopedName(_entity, _ecm) + "/nmea";
    data.SetTopic(topic);
  }

  auto sensor = this->sensorFactory.CreateSensor<NmeaGpsSensor>(data);
  if (nullptr == sensor) {
    gzerr2 << "Failed to create sensor [" << sensorScopedName << "]" << std::endl;
    return;
  }

  // set sensor parent
  std::string parentName = _ecm.Component<::gz::sim::components::Name>(_parent->Data())->Data();
  sensor->SetParent(parentName);

  this->entitySensorMap.insert(std::make_pair(_entity, std::move(sensor)));
  this->newSensors.insert(_entity);
}

//////////////////////////////////////////////////
void NmeaGps::Implementation::CreateSensors(const ::gz::sim::EntityComponentManager & _ecm)
{
  GZ_PROFILE("NmeaGps::CreateSensors");

  if (!this->initialized) {
    _ecm.Each<::gz::sim::components::CustomSensor, ::gz::sim::components::ParentEntity>(
      [&](
        const ::gz::sim::Entity & _entity,
        const ::gz::sim::components::CustomSensor * _custom,
        const ::gz::sim::components::ParentEntity * _parent) -> bool {
        this->AddSensor(_ecm, _entity, _custom, _parent);
        return true;
      });
    this->initialized = true;
  } else {
    _ecm.EachNew<::gz::sim::components::CustomSensor, ::gz::sim::components::ParentEntity>(
      [&](
        const ::gz::sim::Entity & _entity,
        const ::gz::sim::components::CustomSensor * _custom,
        const ::gz::sim::components::ParentEntity * _parent) -> bool {
        this->AddSensor(_ecm, _entity, _custom, _parent);
        return true;
      });
  }
}

//////////////////////////////////////////////////
void NmeaGps::Implementation::Update(const ::gz::sim::EntityComponentManager & _ecm)
{
  GZ_PROFILE("NmeaGps::Update");

  _ecm.Each<::gz::sim::components::CustomSensor, ::gz::sim::components::WorldLinearVelocity>(
    [&](
      const ::gz::sim::Entity & _entity,
      const ::gz::sim::components::CustomSensor * /*_custom*/,
      const ::gz::sim::components::WorldLinearVelocity * _worldLinearVel) -> bool {
      auto it = this->entitySensorMap.find(_entity);

      if (it == this->entitySensorMap.end()) {
        gzerr2 << "Failed to update NmeaGps sensor entity [" << _entity << "]. Entity not found."
               << std::endl;
        return true;
      }

      // Position
      auto latLonEle = sphericalCoordinates(_entity, _ecm);
      if (!latLonEle) {
        gzwarn2 << "Failed to update NmeaGps sensor entity [" << _entity
                << "]. Spherical coordinates not set." << std::endl;
        return true;
      }

      it->second->SetLatitude(GZ_DTOR(latLonEle.value().X()));
      it->second->SetLongitude(GZ_DTOR(latLonEle.value().Y()));
      it->second->SetAltitude(latLonEle.value().Z());

      // Yaw
      auto xyzPose = worldPose(_entity, _ecm);
      it->second->SetYaw(xyzPose.Yaw());

      // Velocity in ENU frame
      it->second->SetVelocity(_worldLinearVel->Data());

      return true;
    });
}

//////////////////////////////////////////////////
void NmeaGps::Implementation::RemoveSensors(const ::gz::sim::EntityComponentManager & _ecm)
{
  GZ_PROFILE("NmeaGps::RemoveSensors");
  _ecm.EachRemoved<::gz::sim::components::CustomSensor>(
    [&](const ::gz::sim::Entity & _entity, const ::gz::sim::components::CustomSensor *) -> bool {
      auto sensorId = this->entitySensorMap.find(_entity);
      if (sensorId == this->entitySensorMap.end()) {
        gzerr2 << "Internal error, missing NmeaGps sensor for entity [" << _entity << "]"
               << std::endl;
        return true;
      }

      this->entitySensorMap.erase(sensorId);

      return true;
    });
}

GZ_ADD_PLUGIN(NmeaGps, ::gz::sim::System, NmeaGps::ISystemPreUpdate, NmeaGps::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(NmeaGps, "romea::gz::NmeaGps")

}  // namespace gz
}  // namespace romea
