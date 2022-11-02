#ifndef __GpsDataDiagnostic_HPP__
#define __GpsDataDiagnostic_HPP__

//romea
#include <romea_core_common/diagnostic/CheckupRate.hpp>

namespace romea {

class GpsDataDiagnostics
{
public :

  GpsDataDiagnostics(const double & desired_rate);

  void updateGGARate(const romea::Duration & duration);
  void updateRMCRate(const romea::Duration & duration);
  void updateGSVRate(const romea::Duration & duration);

  DiagnosticReport makeReport(const romea::Duration & duration);

private :

  CheckupEqualToRate  gga_rate_diagnostic_;
  CheckupEqualToRate  rmc_rate_diagnostic_;
  CheckupEqualToRate  gsv_rate_diagnostic_;

};

}
#endif
