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

class GpsDataDiagnostics
{
public:
  explicit GpsDataDiagnostics(const double & desired_rate);

  void updateGGARate(const romea::Duration & duration);
  void updateRMCRate(const romea::Duration & duration);
  void updateGSVRate(const romea::Duration & duration);

  DiagnosticReport makeReport(const romea::Duration & duration);

private:
  CheckupEqualToRate gga_rate_diagnostic_;
  CheckupEqualToRate rmc_rate_diagnostic_;
  CheckupGreaterThanRate gsv_rate_diagnostic_;
};

}  // namespace romea

#endif  // ROMEA_GPS_UTILS__GPS_DATA_DIAGNOSTICS_HPP_
