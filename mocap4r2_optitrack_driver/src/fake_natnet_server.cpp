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
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "mocap4r2_optitrack_driver/fake_natnet_server.hpp"

namespace mocap4r2_optitrack_driver
{

namespace
{

bool same(const sockaddr_in & a, const sockaddr_in & b)
{
  return a.sin_addr.s_addr == b.sin_addr.s_addr && a.sin_port == b.sin_port;
}

}  // namespace

FakeNatNetServer::FakeNatNetServer(
  const Config & config, const natnet::DataDescriptions & descriptions)
: config_(config), descriptions_(descriptions)
{
  command_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  data_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (command_fd_ < 0 || data_fd_ < 0) {
    throw std::runtime_error(std::string("socket: ") + std::strerror(errno));
  }
  int one = 1;
  setsockopt(command_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(config_.command_port);
  if (inet_pton(AF_INET, config_.address.c_str(), &addr.sin_addr) != 1 ||
    bind(command_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
  {
    const std::string err = std::strerror(errno);
    close(command_fd_);
    close(data_fd_);
    throw std::runtime_error("cannot bind NatNet command port: " + err);
  }
  if (config_.multicast) {
    in_addr iface{};
    inet_pton(AF_INET, config_.address.c_str(), &iface);
    setsockopt(data_fd_, IPPROTO_IP, IP_MULTICAST_IF, &iface, sizeof(iface));
    unsigned char loop = 1;
    setsockopt(data_fd_, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop));
  }
  thread_ = std::thread(&FakeNatNetServer::command_loop, this);
}

FakeNatNetServer::~FakeNatNetServer()
{
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
  close(command_fd_);
  close(data_fd_);
}

void
FakeNatNetServer::set_descriptions(const natnet::DataDescriptions & d)
{
  std::lock_guard<std::mutex> lock(mutex_);
  descriptions_ = d;
}

size_t
FakeNatNetServer::clients() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return clients_.size();
}

natnet::Version
FakeNatNetServer::requested_version() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return requested_;
}

void
FakeNatNetServer::command_loop()
{
  std::vector<uint8_t> buf(65507);
  while (running_) {
    pollfd p{command_fd_, POLLIN, 0};
    if (poll(&p, 1, 50) <= 0) {
      continue;
    }
    sockaddr_in from{};
    socklen_t len = sizeof(from);
    const ssize_t n = recvfrom(
      command_fd_, buf.data(), buf.size(), 0, reinterpret_cast<sockaddr *>(&from), &len);
    if (n < 4) {
      continue;
    }
    const auto h = natnet::parse_header(buf.data(), static_cast<size_t>(n));
    std::vector<uint8_t> reply;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      switch (h.id) {
        case natnet::MSG_CONNECT: {
            connects_++;
            // The official client receives unicast data where it connects from;
            // the open one registers its data socket with keep-alives.
            bool known = false;
            for (const auto & c : clients_) {
              known |= same(c, from);
            }
            if (!known) {
              clients_.push_back(from);
            }
            if (n >= 4 + 267) {
              requested_ = {buf[4 + 265], buf[4 + 266]};
            }
            natnet::ServerInfo info;
            info.app_name = "FakeMotive";
            info.app_version = {3, 1, 0, 0};
            info.natnet_version = {static_cast<uint8_t>(config_.version.major),
              static_cast<uint8_t>(config_.version.minor), 0, 0};
            info.clock_frequency = config_.clock_frequency;
            info.data_port = config_.data_port;
            info.multicast = config_.multicast;
            inet_pton(AF_INET, config_.multicast_address.c_str(), info.multicast_address.data());
            info.has_connection_info = true;
            reply = natnet::encode_server_info(info);
            break;
          }
        case natnet::MSG_REQUEST_MODELDEF:
          descriptions_requests_++;
          reply = natnet::encode_model_def(descriptions_, config_.version);
          break;
        case natnet::MSG_KEEPALIVE: {
            bool known = false;
            for (const auto & c : clients_) {
              known |= same(c, from);
            }
            if (!known) {
              clients_.push_back(from);
            }
            break;
          }
        case natnet::MSG_DISCONNECT:
          clients_.clear();
          break;
        default:
          break;
      }
    }
    if (!reply.empty()) {
      sendto(command_fd_, reply.data(), reply.size(), 0,
        reinterpret_cast<sockaddr *>(&from), len);
    }
  }
}

void
FakeNatNetServer::send_raw(const std::vector<uint8_t> & bytes)
{
  if (config_.multicast) {
    sockaddr_in to{};
    to.sin_family = AF_INET;
    to.sin_port = htons(config_.data_port);
    inet_pton(AF_INET, config_.multicast_address.c_str(), &to.sin_addr);
    sendto(data_fd_, bytes.data(), bytes.size(), 0, reinterpret_cast<sockaddr *>(&to), sizeof(to));
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & c : clients_) {
    sendto(data_fd_, bytes.data(), bytes.size(), 0,
      reinterpret_cast<const sockaddr *>(&c), sizeof(c));
  }
}

void
FakeNatNetServer::send_frame(const natnet::FrameData & frame)
{
  send_raw(natnet::encode_frame(frame, config_.version));
}

}  // namespace mocap4r2_optitrack_driver
