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

#ifndef MOCAP4R2_OPTITRACK_DRIVER__NATNET_CLIENT_HPP_
#define MOCAP4R2_OPTITRACK_DRIVER__NATNET_CLIENT_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "mocap4r2_optitrack_driver/natnet_backend.hpp"
#include "mocap4r2_optitrack_driver/natnet_protocol.hpp"

namespace mocap4r2_optitrack_driver
{

// Open implementation of the NatNet client over UDP. Independent of ROS.
class NatNetClient : public NatNetBackend
{
public:
  using Config = NatNetConfig;
  using Connection = NatNetConfig::Connection;

  explicit NatNetClient(const Config & config);
  ~NatNetClient() override;

  NatNetClient(const NatNetClient &) = delete;
  NatNetClient & operator=(const NatNetClient &) = delete;

  // Ask the server for its info. Throws std::runtime_error on timeout or socket errors.
  natnet::ServerInfo connect() override;

  // Ask for the data descriptions. Requires connect().
  natnet::DataDescriptions request_descriptions() override;

  // Open the data socket and start the receive thread.
  void start(FrameCallback on_frame, ErrorCallback on_error = nullptr) override;
  void stop() override;
  bool running() const {return running_;}

  // Stop and close everything (idempotent).
  void disconnect() override;

  const natnet::Version & version() const {return version_;}
  bool multicast() const override {return multicast_;}
  std::string name() const override {return "open";}

private:
  std::vector<uint8_t> command(const std::vector<uint8_t> & request, uint16_t expected);
  void open_data_socket();
  void receive_loop();

  Config config_;
  int command_fd_{-1};
  int data_fd_{-1};
  natnet::ServerInfo info_;
  natnet::Version version_;
  bool multicast_{true};
  std::mutex command_mutex_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  FrameCallback on_frame_;
  ErrorCallback on_error_;
};

}  // namespace mocap4r2_optitrack_driver

#endif  // MOCAP4R2_OPTITRACK_DRIVER__NATNET_CLIENT_HPP_
