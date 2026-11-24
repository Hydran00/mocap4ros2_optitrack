// Copyright 2026 Intelligent Robotics Lab
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

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mocap4r2_optitrack_driver/natnet_client.hpp"

namespace mocap4r2_optitrack_driver
{

namespace
{

constexpr size_t kMaxPacket = 65507;
constexpr uint16_t kDefaultDataPort = 1511;
constexpr char kDefaultMulticast[] = "239.255.42.99";
// Bitstream requested in unicast: newest one this parser knows.
constexpr natnet::Version kRequestedVersion{4, 4};

in_addr to_addr(const std::string & ip, const char * what)
{
  in_addr a{};
  if (inet_pton(AF_INET, ip.c_str(), &a) != 1) {
    throw std::invalid_argument(std::string("invalid ") + what + " '" + ip + "'");
  }
  return a;
}

sockaddr_in endpoint(const in_addr & a, uint16_t port)
{
  sockaddr_in s{};
  s.sin_family = AF_INET;
  s.sin_addr = a;
  s.sin_port = htons(port);
  return s;
}

std::runtime_error sys_error(const std::string & what)
{
  return std::runtime_error(what + ": " + std::strerror(errno));
}

int udp_socket(const in_addr & bind_addr, uint16_t port, bool reuse)
{
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    throw sys_error("socket");
  }
  if (reuse) {
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  }
  int rcvbuf = 4 * 1024 * 1024;  // absorb bursts at high frame rates
  setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
  auto addr = endpoint(bind_addr, port);
  if (bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    const auto e = sys_error("bind " + std::to_string(port));
    close(fd);
    throw e;
  }
  return fd;
}

void send_to(int fd, const std::vector<uint8_t> & buf, const sockaddr_in & to)
{
  if (sendto(fd, buf.data(), buf.size(), 0,
    reinterpret_cast<const sockaddr *>(&to), sizeof(to)) < 0)
  {
    throw sys_error("sendto");
  }
}

}  // namespace

NatNetClient::NatNetClient(const Config & config)
: config_(config)
{
  to_addr(config_.server_address, "server_address");
  to_addr(config_.local_address, "local_address");
  if (!config_.multicast_address.empty()) {
    to_addr(config_.multicast_address, "multicast_address");
  }
}

NatNetClient::~NatNetClient()
{
  disconnect();
}

std::vector<uint8_t>
NatNetClient::command(const std::vector<uint8_t> & request, uint16_t expected)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  const auto server = endpoint(to_addr(config_.server_address, "server_address"),
      config_.command_port);
  send_to(command_fd_, request, server);

  std::vector<uint8_t> buf(kMaxPacket);
  const auto deadline = std::chrono::steady_clock::now() + config_.timeout;
  while (true) {
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now()).count();
    if (left <= 0) {
      throw std::runtime_error(
              "no answer from NatNet server " + config_.server_address + ":" +
              std::to_string(config_.command_port));
    }
    pollfd p{command_fd_, POLLIN, 0};
    const int r = poll(&p, 1, static_cast<int>(left));
    if (r < 0 && errno != EINTR) {
      throw sys_error("poll");
    }
    if (r <= 0) {
      continue;
    }
    const ssize_t n = recv(command_fd_, buf.data(), buf.size(), 0);
    if (n < 4) {
      continue;
    }
    // Other messages (strings, frames in unicast) may arrive on this socket.
    if (natnet::parse_header(buf.data(), static_cast<size_t>(n)).id == expected) {
      buf.resize(static_cast<size_t>(n));
      return buf;
    }
  }
}

natnet::ServerInfo
NatNetClient::connect()
{
  if (command_fd_ < 0) {
    command_fd_ = udp_socket(to_addr(config_.local_address, "local_address"), 0, false);
  }
  const auto reply = command(natnet::make_connect_request(kRequestedVersion),
      natnet::MSG_SERVERINFO);
  info_ = natnet::parse_server_info(reply.data(), reply.size());
  version_ = info_.version();
  if (version_.major < 3) {
    throw std::runtime_error(
            "NatNet " + std::to_string(version_.major) + "." + std::to_string(version_.minor) +
            " is not supported (3.0 or newer required)");
  }
  switch (config_.connection) {
    case Connection::MULTICAST: multicast_ = true; break;
    case Connection::UNICAST: multicast_ = false; break;
    case Connection::AUTO: multicast_ = info_.has_connection_info ? info_.multicast : true; break;
  }
  return info_;
}

natnet::DataDescriptions
NatNetClient::request_descriptions()
{
  if (command_fd_ < 0) {
    throw std::runtime_error("not connected");
  }
  const auto reply = command(
    natnet::make_simple_request(natnet::MSG_REQUEST_MODELDEF), natnet::MSG_MODELDEF);
  return natnet::parse_model_def(reply.data(), reply.size(), version_);
}

void
NatNetClient::open_data_socket()
{
  const auto local = to_addr(config_.local_address, "local_address");
  if (multicast_) {
    uint16_t port = config_.data_port;
    if (port == 0) {
      port = info_.has_connection_info && info_.data_port ? info_.data_port : kDefaultDataPort;
    }
    std::string group = config_.multicast_address;
    if (group.empty()) {
      char buf[INET_ADDRSTRLEN];
      group = info_.has_connection_info ?
        inet_ntop(AF_INET, info_.multicast_address.data(), buf, sizeof(buf)) : kDefaultMulticast;
    }
    data_fd_ = udp_socket(in_addr{INADDR_ANY}, port, true);
    ip_mreq mreq{};
    mreq.imr_multiaddr = to_addr(group, "multicast_address");
    mreq.imr_interface = local;
    if (setsockopt(data_fd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
      const auto e = sys_error("join multicast group " + group);
      close(data_fd_);
      data_fd_ = -1;
      throw e;
    }
  } else {
    // Unicast: the server streams to the address that sends keep-alives.
    data_fd_ = udp_socket(local, 0, false);
  }
}

void
NatNetClient::start(FrameCallback on_frame, ErrorCallback on_error)
{
  if (running_) {
    return;
  }
  if (command_fd_ < 0) {
    throw std::runtime_error("not connected");
  }
  on_frame_ = std::move(on_frame);
  on_error_ = std::move(on_error);
  if (data_fd_ < 0) {
    open_data_socket();
  }
  running_ = true;
  thread_ = std::thread(&NatNetClient::receive_loop, this);
}

void
NatNetClient::receive_loop()
{
  std::vector<uint8_t> buf(kMaxPacket);
  const auto server = endpoint(to_addr(config_.server_address, "server_address"),
      config_.command_port);
  const auto keepalive = natnet::make_simple_request(natnet::MSG_KEEPALIVE);
  auto next_keepalive = std::chrono::steady_clock::now();

  while (running_) {
    if (!multicast_ && std::chrono::steady_clock::now() >= next_keepalive) {
      try {
        send_to(data_fd_, keepalive, server);
      } catch (const std::exception & e) {
        if (on_error_) {on_error_(e.what());}
      }
      next_keepalive = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    }
    pollfd p{data_fd_, POLLIN, 0};
    const int r = poll(&p, 1, 100);
    if (r <= 0) {
      continue;
    }
    const ssize_t n = recv(data_fd_, buf.data(), buf.size(), 0);
    if (n < 4) {
      continue;
    }
    try {
      const auto h = natnet::parse_header(buf.data(), static_cast<size_t>(n));
      if (h.id != natnet::MSG_FRAMEOFDATA) {
        continue;
      }
      auto frame = natnet::parse_frame(buf.data(), static_cast<size_t>(n), version_);
      if (on_frame_) {
        on_frame_(std::move(frame));
      }
    } catch (const natnet::ParseError & e) {
      if (on_error_) {on_error_(std::string("bad NatNet frame: ") + e.what());}
    }
  }
}

void
NatNetClient::stop()
{
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
}

void
NatNetClient::disconnect()
{
  stop();
  if (command_fd_ >= 0) {
    try {
      const auto server = endpoint(to_addr(config_.server_address, "server_address"),
          config_.command_port);
      send_to(command_fd_, natnet::make_simple_request(natnet::MSG_DISCONNECT), server);
    } catch (const std::exception &) {
      // Best effort: the server also drops clients that stop sending keep-alives.
    }
    close(command_fd_);
    command_fd_ = -1;
  }
  if (data_fd_ >= 0) {
    close(data_fd_);
    data_fd_ = -1;
  }
}

}  // namespace mocap4r2_optitrack_driver
