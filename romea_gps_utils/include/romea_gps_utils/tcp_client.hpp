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

#ifndef ROMEA_GPS_UTILS__TCP_CLIENT_HPP_
#define ROMEA_GPS_UTILS__TCP_CLIENT_HPP_

#include <string>
#include <sstream>
#include <vector>

namespace romea
{
namespace ros2
{

class TcpClient
{
public:
  explicit TcpClient(long timeout_ms) // NOLINT
  : timeout_{timeout_ms}, socket_{-1} {}
  ~TcpClient();

  void connect(std::string const & ip, int port);
  std::size_t send(const std::vector<std::uint8_t> & data) const;
  std::string readline();

private:
  void recvlines();

private:
  long timeout_; // NOLINT
  int socket_;
  std::stringstream buffer_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_GPS_UTILS__TCP_CLIENT_HPP_
