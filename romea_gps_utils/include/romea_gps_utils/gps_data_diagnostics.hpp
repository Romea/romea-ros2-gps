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


#ifndef ROMEA_GPS_UTILS__GPS_DATA_DIAGNOSTICS_HPP_
#define ROMEA_GPS_UTILS__GPS_DATA_DIAGNOSTICS_HPP_

// romea core
#include "romea_core_common/diagnostic/CheckupRate.hpp"

namespace romea
{
namespace ros2
{

class GpsDataDiagnostics
{
public:
  explicit GpsDataDiagnostics(const double & desired_rate);

  void updateGGARate(const core::Duration & duration);
  void updateRMCRate(const core::Duration & duration);
  void updateGSVRate(const core::Duration & duration);

  core::DiagnosticReport makeReport(const core::Duration & duration);

private:
  core::CheckupEqualToRate gga_rate_diagnostic_;
  core::CheckupEqualToRate rmc_rate_diagnostic_;
  core::CheckupGreaterThanRate gsv_rate_diagnostic_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_GPS_UTILS__GPS_DATA_DIAGNOSTICS_HPP_
