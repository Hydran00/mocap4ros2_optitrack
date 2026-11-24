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

#ifndef MOCAP4R2_OPTITRACK_DRIVER__FAKE_NATNET_SERVER_HPP_
#define MOCAP4R2_OPTITRACK_DRIVER__FAKE_NATNET_SERVER_HPP_

#include <netinet/in.h>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "mocap4r2_optitrack_driver/natnet_protocol.hpp"

namespace mocap4r2_optitrack_driver
{

// Minimal NatNet server (a stand-in for Motive) for tests and setup checks.
class FakeNatNetServer
{
public:
  struct Config
  {
    std::string address{"127.0.0.1"};         // command socket address
    uint16_t command_port{1510};
    uint16_t data_port{1511};
    bool multicast{false};
    std::string multicast_address{"239.255.42.99"};
    natnet::Version version{4, 4};
    uint64_t clock_frequency{10000000};
  };

  FakeNatNetServer(const Config & config, const natnet::DataDescriptions & descriptions);
  ~FakeNatNetServer();

  // Send a frame to the clients (unicast) or the multicast group.
  void send_frame(const natnet::FrameData & frame);
  // Send raw bytes as a frame (malformed data tests).
  void send_raw(const std::vector<uint8_t> & bytes);

  void set_descriptions(const natnet::DataDescriptions & d);
  size_t clients() const;
  int connect_requests() const {return connects_;}
  int description_requests() const {return descriptions_requests_;}
  natnet::Version requested_version() const;

private:
  void command_loop();

  Config config_;
  int command_fd_{-1};
  int data_fd_{-1};
  mutable std::mutex mutex_;
  natnet::DataDescriptions descriptions_;
  std::vector<sockaddr_in> clients_;
  natnet::Version requested_;
  std::atomic<int> connects_{0};
  std::atomic<int> descriptions_requests_{0};
  std::atomic<bool> running_{true};
  std::thread thread_;
};

}  // namespace mocap4r2_optitrack_driver

#endif  // MOCAP4R2_OPTITRACK_DRIVER__FAKE_NATNET_SERVER_HPP_
