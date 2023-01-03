// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_GPS_UTILS__GPS_DATA_DIAGNOSTICS_HPP_
#define ROMEA_GPS_UTILS__GPS_DATA_DIAGNOSTICS_HPP_

// romea core
#include <romea_core_common/diagnostic/CheckupRate.hpp>

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
