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

#ifndef ROMEA_GPS_GAZEBO__NMEA_GPS_SYSTEM_HPP_
#define ROMEA_GPS_GAZEBO__NMEA_GPS_SYSTEM_HPP_

#include "gz/sim/System.hh"
#include "gz/sim/config.hh"
#include "gz/utils/ImplPtr.hh"

namespace romea
{
namespace gz
{
class NmeaGps : public ::gz::sim::System,
                public ::gz::sim::ISystemPreUpdate,
                public ::gz::sim::ISystemPostUpdate
{
public:
  NmeaGps();

  void PreUpdate(
    const ::gz::sim::UpdateInfo & _info, ::gz::sim::EntityComponentManager & _ecm) final;

  void PostUpdate(
    const ::gz::sim::UpdateInfo & _info, const ::gz::sim::EntityComponentManager & _ecm) final;

  GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
};

}  // namespace gz
}  // namespace romea

#endif  // ROMEA_GPS_GAZEBO__NMEA_GPS_SYSTEM_HPP_
