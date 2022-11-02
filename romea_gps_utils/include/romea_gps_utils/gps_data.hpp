#ifndef _romea_GpsData_hpp_
#define _romea_GpsData_hpp_

#include "gps_data_conversions.hpp"
#include "gps_data_diagnostics.hpp"

#include "romea_common_utils/publishers/stamped_data_publisher.hpp"
#include "romea_common_utils/publishers/diagnostic_publisher.hpp"

#include <serial/serial.h>

namespace romea {

class GpsData
{
public:

  GpsData(std::shared_ptr<rclcpp::Node> node);

  void process_nmea_sentence(const std::string & nmea_sentence);

protected:

  void init_diagnostics_(std::shared_ptr<rclcpp::Node> node);

  void init_publishers_(std::shared_ptr<rclcpp::Node> node);

  void init_timer_(std::shared_ptr<rclcpp::Node> node);


  void process_gga_frame_(const rclcpp::Time &stamp,
                          const std::string & nmea_sentence);

  void process_rmc_frame_(const rclcpp::Time &stamp,
                          const std::string & nmea_sentence);

  void process_gsv_frame_(const rclcpp::Time &stamp,
                          const std::string & nmea_sentence);


  bool can_be_converted_to_fix_msg_(const GGAFrame & gga_frame);

  bool can_be_converted_to_vel_msg_(const RMCFrame & rmc_frame);

  void timer_callback_();

protected:

  std::shared_ptr<rclcpp::Clock> clock_;
  std::shared_ptr<rclcpp::TimerBase> timer_;

  std::unique_ptr<GpsDataDiagnostics> diagnostics_;
  std::shared_ptr<StampedPublisherBase<GGAFrame>> fix_publisher_;
  std::shared_ptr<StampedPublisherBase<RMCFrame>> vel_publisher_;
  std::shared_ptr<StampedPublisherBase<std::string>> nmea_sentence_publisher_;
  std::shared_ptr<StampedPublisherBase<DiagnosticReport>> diagnostics_publisher_;
};

}

#endif
