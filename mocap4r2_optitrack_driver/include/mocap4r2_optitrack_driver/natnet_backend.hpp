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

#ifndef MOCAP4R2_OPTITRACK_DRIVER__NATNET_BACKEND_HPP_
#define MOCAP4R2_OPTITRACK_DRIVER__NATNET_BACKEND_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "mocap4r2_optitrack_driver/natnet_protocol.hpp"

namespace mocap4r2_optitrack_driver
{

struct NatNetConfig
{
  enum class Connection {AUTO, MULTICAST, UNICAST};

  std::string server_address{"127.0.0.1"};
  std::string local_address{"0.0.0.0"};    // interface for multicast and sockets
  uint16_t command_port{1510};
  uint16_t data_port{0};                   // 0: as announced by the server (default 1511)
  std::string multicast_address;           // empty: as announced (default 239.255.42.99)
  Connection connection{Connection::AUTO};
  std::chrono::milliseconds timeout{2000};
};

// Connection to Motive. Two implementations: the official NatNet SDK (when the
// package was built with it) and an open implementation of the protocol.
// Both deliver the same data structures.
class NatNetBackend
{
public:
  // Called from the backend thread for every frame.
  using FrameCallback = std::function<void (natnet::FrameData && frame)>;
  // Called from the backend thread when data cannot be decoded.
  using ErrorCallback = std::function<void (const std::string & error)>;

  virtual ~NatNetBackend() = default;

  // Connect and return the server info. Throws std::runtime_error on failure.
  virtual natnet::ServerInfo connect() = 0;
  // Data descriptions (asset names and structure). Throws on failure.
  virtual natnet::DataDescriptions request_descriptions() = 0;
  // Start delivering frames.
  virtual void start(FrameCallback on_frame, ErrorCallback on_error = nullptr) = 0;
  // Stop delivering frames (the connection is kept).
  virtual void stop() = 0;
  // Close the connection (idempotent).
  virtual void disconnect() = 0;

  virtual bool multicast() const = 0;
  virtual std::string name() const = 0;
};

enum class BackendKind {AUTO, SDK, OPEN};

// True if this build includes the official NatNet SDK backend.
bool sdk_backend_available();

// AUTO picks the SDK when available. Throws std::invalid_argument when SDK is
// requested but this build has no SDK, or for invalid addresses.
std::unique_ptr<NatNetBackend> make_backend(BackendKind kind, const NatNetConfig & config);

}  // namespace mocap4r2_optitrack_driver

#endif  // MOCAP4R2_OPTITRACK_DRIVER__NATNET_BACKEND_HPP_
