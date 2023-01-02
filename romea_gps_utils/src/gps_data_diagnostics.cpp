// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#include "romea_gps_utils/gps_data_diagnostics.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
GpsDataDiagnostics::GpsDataDiagnostics(const double & desired_rate)
: gga_rate_diagnostic_("gga_rate", desired_rate, desired_rate * 0.1),
  rmc_rate_diagnostic_("rmc_rate", desired_rate, desired_rate * 0.1),
  gsv_rate_diagnostic_("gsv_rate", desired_rate, desired_rate * 0.1)
{
}

//-----------------------------------------------------------------------------
void GpsDataDiagnostics::updateGGARate(const romea::Duration & duration)
{
  gga_rate_diagnostic_.evaluate(duration);
}

//-----------------------------------------------------------------------------
void GpsDataDiagnostics::updateRMCRate(const romea::Duration & duration)
{
  rmc_rate_diagnostic_.evaluate(duration);
}

//-----------------------------------------------------------------------------
void GpsDataDiagnostics::updateGSVRate(const romea::Duration & duration)
{
  gsv_rate_diagnostic_.evaluate(duration);
}

//-----------------------------------------------------------------------------
DiagnosticReport GpsDataDiagnostics::makeReport(const Duration & duration)
{
  gga_rate_diagnostic_.heartBeatCallback(duration);
  rmc_rate_diagnostic_.heartBeatCallback(duration);
  gsv_rate_diagnostic_.heartBeatCallback(duration);

  DiagnosticReport report;
  report += gga_rate_diagnostic_.getReport();
  report += rmc_rate_diagnostic_.getReport();
  report += gsv_rate_diagnostic_.getReport();
  return report;
}

}  // namespace romea
