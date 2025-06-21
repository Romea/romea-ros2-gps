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

#include "romea_gps_utils/tcp_client.hpp"

#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <strings.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace romea::ros2
{

TcpClient::~TcpClient()
{
  if (socket_ >= 0) {
    ::close(socket_);
  }
}

void TcpClient::connect(const std::string & ip, int port)
{
  struct hostent * hostent;
  struct sockaddr_in addr;

  addr.sin_port = htons(port);
  addr.sin_family = AF_INET;
  hostent = static_cast<struct hostent *>(gethostbyname(ip.c_str()));

  if (!hostent) {
    throw std::runtime_error("Invalid IP address");
  }

  bcopy(
    reinterpret_cast<char *>(hostent->h_addr),
    reinterpret_cast<char *>(&addr.sin_addr),
    hostent->h_length);

  socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_ == -1) {
    throw std::runtime_error("Cannot create socket");
  }

  if (
    ::connect(socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(struct sockaddr_in)) <
    0) {
    int err = errno;
    socket_ = -1;
    throw std::runtime_error(
      "Cannot connect to " + ip + ':' + std::to_string(port) + ": " + strerror(err));
  }
}

std::size_t TcpClient::send(const std::vector<std::uint8_t> & data) const
{
  std::size_t sent;
  const std::uint8_t * str = data.data();
  for (sent = 0; sent < data.size();) {
    int r = ::send(socket_, str + sent, data.size() - sent, 0);
    if (r == -1) {
      throw std::runtime_error("Sending data failed");
    }
    sent += r;
  }
  return sent;
}

std::string TcpClient::readline()
{
  for (;;) {
    if (auto line = try_extract_line()) {
      return *line;
    }

    recv_buf();
  }
}

void TcpClient::recv_buf()
{
  timeval timeout{timeout_ / 1000, (timeout_ % 1000) * 1000};
  fd_set set;
  FD_ZERO(&set);
  FD_SET(socket_, &set);

  int ret = select(socket_ + 1, &set, NULL, NULL, &timeout);
  if (ret == 0) {
    throw std::runtime_error{"TCP receive: timeout"};
  }
  if (ret < 0) {
    throw std::runtime_error{std::string{"TCP receive: select error: "} + strerror(errno)};
  }

  char rstr[BUFSIZ];
  ssize_t size = ::recv(socket_, rstr, BUFSIZ, 0);
  if (size < 0) {
    throw std::runtime_error{std::string{"TCP receive: recv error: "} + strerror(errno)};
  }

  buffer_.append(rstr, size);
}

std::optional<std::string> TcpClient::try_extract_line()
{
  auto pos = buffer_.find('\n', buffer_start_);
  if (pos == std::string::npos) {
    return {};
  }

  std::string line = buffer_.substr(buffer_start_, pos + 1 - buffer_start_);
  buffer_start_ = pos + 1;

  // clear the begining of the buffer when it reach half of its max size
  if (buffer_start_ > BUFSIZ >> 1) {
    buffer_.erase(0, buffer_start_);
    buffer_start_ = 0;
  }

  return line;
}

}  // namespace romea::ros2
