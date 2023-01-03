// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license


#ifndef ROMEA_GPS_UTILS__GPS_SERIAL_INTERFACE_HPP_
#define ROMEA_GPS_UTILS__GPS_SERIAL_INTERFACE_HPP_

// ros
#include <rclcpp/rclcpp.hpp>

// serial
#include <serial/serial.h>

// std
#include <optional>
#include <memory>
#include <string>

namespace romea
{

class GpsSerialInterface
{
public:
  explicit GpsSerialInterface(std::shared_ptr<rclcpp::Node> node);

  size_t read(std::string & data);

  size_t write(const std::string & data);

  std::optional<std::string> read_nmea_sentence();

protected:
  serial::Serial serial_;
  std::string device_;
  int baudrate_;
};

}  // namespace romea

#endif  // ROMEA_GPS_UTILS__GPS_SERIAL_INTERFACE_HPP_
