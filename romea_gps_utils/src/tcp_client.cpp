#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdexcept>
#include <strings.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>

#include "romea_gps_utils/tcp_client.hpp"

namespace romea {
namespace ros2 {

TcpClient::~TcpClient()
{
  if(socket_ >= 0)
    ::close(socket_);
}

void TcpClient::connect(std::string const & ip, int port)
{
  struct hostent*    hostent;
  struct sockaddr_in addr;

  addr.sin_port   = htons(port);
  addr.sin_family = AF_INET;
  hostent         = static_cast<struct hostent*>(gethostbyname(ip.c_str()));

  if(!hostent) throw std::runtime_error("Invalid IP address");

  bcopy(
    reinterpret_cast<char*>(hostent->h_addr),
    reinterpret_cast<char*>(&addr.sin_addr),
    hostent->h_length
  );

  socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if(socket_ == -1) throw std::runtime_error("Cannot create socket");

  if(::connect(socket_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(struct sockaddr_in)) < 0)
  {
    int err = errno;
    socket_ = -1;
    throw std::runtime_error("Cannot connect to " + ip + ':' + std::to_string(port) + ": " +
      strerror(err));
  }
}

std::size_t TcpClient::send(const std::vector<std::uint8_t> & data) const
{
  std::size_t sent;
  std::uint8_t const* str = data.data();
  for(sent = 0; sent < data.size();) {
    int r = ::send(socket_, str + sent, data.size() - sent, 0);
    if(r == -1) throw std::runtime_error("Sending data failed");
    sent += r;
  }
  return sent;
}

std::string TcpClient::readline()
{
  if(!buffer_.rdbuf()->in_avail())
    recvlines();

  std::string line;
  std::getline(buffer_, line);
  return line;
}

void TcpClient::recvlines()
{
  struct timeval timeout;
  fd_set set;

  FD_ZERO(&set);
  FD_SET(socket_, &set);

  timeout.tv_sec  = timeout_ / 1000;
  timeout.tv_usec = (timeout_ % 1000) * 1000;

  if(select(socket_ + 1, &set, NULL, NULL, &timeout)) {
    int size;
    char rstr[BUFSIZ];
    do {
      size = ::recv(socket_, rstr, BUFSIZ, 0);
      if(size > 0) buffer_ << rstr;
      if(size < 0) throw std::runtime_error(strerror(errno));
    } while(size > 0 && rstr[size - 1] != '\n');
  }
}

}  // namespace ros2
}  // namespace romea
