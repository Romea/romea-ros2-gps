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


#include "romea_gps_utils/gps_data_diagnostics.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
GpsDataDiagnostics::GpsDataDiagnostics(const double & desired_rate)
: gga_rate_diagnostic_("gga_rate", desired_rate, desired_rate * 0.1),
  rmc_rate_diagnostic_("rmc_rate", desired_rate, desired_rate * 0.1),
  gsv_rate_diagnostic_("gsv_rate", desired_rate, desired_rate * 0.1)
{
}

//-----------------------------------------------------------------------------
void GpsDataDiagnostics::updateGGARate(const core::Duration & duration)
{
  gga_rate_diagnostic_.evaluate(duration);
}

//-----------------------------------------------------------------------------
void GpsDataDiagnostics::updateRMCRate(const core::Duration & duration)
{
  rmc_rate_diagnostic_.evaluate(duration);
}

//-----------------------------------------------------------------------------
void GpsDataDiagnostics::updateGSVRate(const core::Duration & duration)
{
  gsv_rate_diagnostic_.evaluate(duration);
}

//-----------------------------------------------------------------------------
core::DiagnosticReport GpsDataDiagnostics::makeReport(const core::Duration & duration)
{
  gga_rate_diagnostic_.heartBeatCallback(duration);
  rmc_rate_diagnostic_.heartBeatCallback(duration);
  gsv_rate_diagnostic_.heartBeatCallback(duration);

  core::DiagnosticReport report;
  report += gga_rate_diagnostic_.getReport();
  report += rmc_rate_diagnostic_.getReport();
  report += gsv_rate_diagnostic_.getReport();
  return report;
}

}  // namespace ros2
}  // namespace romea
