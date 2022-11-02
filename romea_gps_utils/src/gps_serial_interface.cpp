#include "romea_gps_utils/gps_serial_interface.hpp"

#include <romea_core_gps/nmea/NMEAParsing.hpp>
#include <romea_common_utils/params/sensor_parameters.hpp>

namespace romea
{

//-----------------------------------------------------------------------------
GpsSerialInterface::GpsSerialInterface(std::shared_ptr<rclcpp::Node> node)
{
  declare_device(node);
  declare_baudrate(node);

  serial_.setPort(get_device(node));
  serial_.setBaudrate(get_baudrate(node));
  serial::Timeout t = serial::Timeout::simpleTimeout(40);
  serial_.setTimeout(t);
  serial_.open();

  if(!serial_.isOpen())
  {
    throw(std::runtime_error("Unable to connect to device "+ get_device(node) ));
  }
}

//-----------------------------------------------------------------------------
size_t GpsSerialInterface::read(std::string & data)
{
  return serial_.readline(data,65536,"\n");
}

//-----------------------------------------------------------------------------
size_t GpsSerialInterface::write(const std::string & data)
{
  return serial_.write(data);
}

//-----------------------------------------------------------------------------
std::optional<std::string> GpsSerialInterface::read_nmea_sentence()
{
  std::string raw_sentence;
  if(read(raw_sentence) && NMEAParsing::isNMEASentence(raw_sentence))
  {
    return raw_sentence;
  }
  else
  {
    return std::nullopt;
  }
}

}

