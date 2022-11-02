#include "romea_gps_utils/gps_data.hpp"
#include <romea_core_common/time/Time.hpp>
#include <romea_common_utils/params/sensor_parameters.hpp>
#include <romea_common_utils/qos.hpp>

namespace romea
{

//-----------------------------------------------------------------------------
GpsData::GpsData(std::shared_ptr<rclcpp::Node> node)
{
  declare_rate(node);
  declare_frame_id(node);
  init_publishers_(node);
  init_diagnostics_(node);
  init_timer_(node);
}

//-----------------------------------------------------------------------------
void GpsData::init_diagnostics_(std::shared_ptr<rclcpp::Node> node)
{
  diagnostics_ = std::make_unique<GpsDataDiagnostics>(get_rate(node));

  diagnostics_publisher_ = make_diagnostic_publisher<DiagnosticReport>
      (node,"gps",1.,"","/diagnostics",true);
}

//-----------------------------------------------------------------------------
void GpsData::init_publishers_(std::shared_ptr<rclcpp::Node> node)
{
  auto frame_id = get_frame_id(node);

  fix_publisher_=make_stamped_data_publisher<GGAFrame,sensor_msgs::msg::NavSatFix>(
        node,"fix",frame_id,sensor_data_qos(),true);
  vel_publisher_=make_stamped_data_publisher<RMCFrame,geometry_msgs::msg::TwistStamped>(
        node,"fix",frame_id,sensor_data_qos(),true);
  nmea_sentence_publisher_=make_stamped_data_publisher<std::string,nmea_msgs::msg::Sentence>(
        node,"fix",frame_id,sensor_data_qos(),true);

}

//-----------------------------------------------------------------------------
void GpsData::init_timer_(std::shared_ptr<rclcpp::Node> node)
{
  auto callback = std::bind(&GpsData::timer_callback_,this);
  timer_ = node->create_wall_timer(durationFromSecond(0.1),callback);
}

//-----------------------------------------------------------------------------
bool GpsData::can_be_converted_to_fix_msg_(const GGAFrame & gga_frame)
{
  return gga_frame.latitude
      && gga_frame.longitude
      && gga_frame.altitudeAboveGeoid
      && gga_frame.geoidHeight
      && gga_frame.horizontalDilutionOfPrecision
      && gga_frame.fixQuality;
}

//-----------------------------------------------------------------------------
bool can_be_converted_to_vel_msg_(const RMCFrame & rmc_frame)
{
  return rmc_frame.speedOverGroundInMeterPerSecond
      && rmc_frame.trackAngleTrue;
}

//-----------------------------------------------------------------------------
void GpsData::process_gga_frame_(const rclcpp::Time  & stamp,
                                 const std::string & nmea_sentence)
{
  GGAFrame gga_frame(nmea_sentence);
  diagnostics_->updateGGARate(to_romea_duration(stamp));
  if(can_be_converted_to_fix_msg_(gga_frame))
  {
    fix_publisher_->publish(stamp,gga_frame);
  }
}

//-----------------------------------------------------------------------------
void GpsData::process_rmc_frame_(const rclcpp::Time &stamp,
                                 const std::string & nmea_sentence)
{
  RMCFrame rmc_frame(nmea_sentence);
  diagnostics_->updateRMCRate(to_romea_duration(stamp));
  if(can_be_converted_to_vel_msg_(rmc_frame))
  {
    vel_publisher_->publish(stamp,rmc_frame);
  }
}

//-----------------------------------------------------------------------------
void GpsData::process_gsv_frame_(const rclcpp::Time &stamp,
                                 const std::string & nmea_sentence)
{
  GSVFrame gsv_frame(nmea_sentence);
  if(gsv_frame.sentenceNumber == 0)
  {
    diagnostics_->updateGSVRate(to_romea_duration(stamp));
  }
}

//-----------------------------------------------------------------------------
void GpsData::process_nmea_sentence(const std::string & nmea_sentence)
{
  std::cout << nmea_sentence<<std::endl;

  rclcpp::Time stamp = clock_->now();
  nmea_sentence_publisher_->publish(stamp,nmea_sentence);

  switch(NMEAParsing::extractSentenceId(nmea_sentence))
  {
  case NMEAParsing::SentenceID::GGA:
    process_gga_frame_(stamp,nmea_sentence);
    break;
  case NMEAParsing::SentenceID::RMC:
    process_rmc_frame_(stamp,nmea_sentence);
    break;
  case NMEAParsing::SentenceID::GSV:
    process_rmc_frame_(stamp,nmea_sentence);
    break;
  default:
    break;
  }
}

//-----------------------------------------------------------------------------
void GpsData::timer_callback_()
{
  auto stamp = clock_->now();
  diagnostics_publisher_->publish(stamp,diagnostics_->makeReport(to_romea_duration(stamp)));
}

}

