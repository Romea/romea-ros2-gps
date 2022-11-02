#ifndef _romea_GpsSerialInterface_hpp_
#define _romea_GpsSerialInterface_hpp_

//ros
#include <rclcpp/rclcpp.hpp>

//serial
#include <serial/serial.h>

//std
#include <optional>

namespace romea {

class GpsSerialInterface
{
public:

  GpsSerialInterface(std::shared_ptr<rclcpp::Node> node);

  size_t read(std::string & data);

  size_t write(const std::string & data);

  std::optional<std::string> read_nmea_sentence();

protected:

  serial::Serial serial_;
  std::string device_;
  int baudrate_;

};

}

#endif
