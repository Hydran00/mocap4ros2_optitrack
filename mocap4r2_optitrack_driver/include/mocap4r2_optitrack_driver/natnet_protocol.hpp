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

#ifndef MOCAP4R2_OPTITRACK_DRIVER__NATNET_PROTOCOL_HPP_
#define MOCAP4R2_OPTITRACK_DRIVER__NATNET_PROTOCOL_HPP_

// Open implementation of the OptiTrack NatNet wire protocol (bitstream 3.0 to 4.5;
// newer 4.x bitstreams are read skipping the blocks this code does not know).
// Every read is bounds-checked: malformed packets raise ParseError, never crash.

#include <array>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace mocap4r2_optitrack_driver
{
namespace natnet
{

// Message ids
constexpr uint16_t MSG_CONNECT = 0;
constexpr uint16_t MSG_SERVERINFO = 1;
constexpr uint16_t MSG_REQUEST = 2;
constexpr uint16_t MSG_RESPONSE = 3;
constexpr uint16_t MSG_REQUEST_MODELDEF = 4;
constexpr uint16_t MSG_MODELDEF = 5;
constexpr uint16_t MSG_REQUEST_FRAMEOFDATA = 6;
constexpr uint16_t MSG_FRAMEOFDATA = 7;
constexpr uint16_t MSG_MESSAGESTRING = 8;
constexpr uint16_t MSG_DISCONNECT = 9;
constexpr uint16_t MSG_KEEPALIVE = 10;
constexpr uint16_t MSG_UNRECOGNIZED_REQUEST = 100;

// Data description types
constexpr int32_t DESC_MARKERSET = 0;
constexpr int32_t DESC_RIGIDBODY = 1;
constexpr int32_t DESC_SKELETON = 2;

// Frame and element flags
constexpr int16_t RB_TRACKING_VALID = 0x01;
constexpr int16_t MARKER_OCCLUDED = 0x01;
constexpr int16_t MARKER_UNLABELED = 0x10;
constexpr int16_t FRAME_MODELS_CHANGED = 0x02;

class ParseError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

struct Version
{
  int major{0};
  int minor{0};

  bool at_least(int ma, int mi) const {return major > ma || (major == ma && minor >= mi);}
  // From 4.1 every data block carries its size in bytes.
  bool has_block_sizes() const {return at_least(4, 1);}
};

struct ServerInfo
{
  std::string app_name;
  std::array<uint8_t, 4> app_version{};
  std::array<uint8_t, 4> natnet_version{};
  uint64_t clock_frequency{0};   // ticks per second of the high resolution timestamps
  uint16_t data_port{0};
  bool multicast{false};
  std::array<uint8_t, 4> multicast_address{};
  bool has_connection_info{false};  // data port and multicast fields present (Motive 3+)

  Version version() const {return {natnet_version[0], natnet_version[1]};}
};

struct RigidBodyDescription
{
  std::string name;
  int32_t id{0};
  int32_t parent_id{-1};
  std::array<float, 3> offset{};
  std::array<float, 4> orientation{0.0f, 0.0f, 0.0f, 1.0f};  // 4.2+
  std::vector<std::string> marker_names;
};

struct SkeletonDescription
{
  std::string name;
  int32_t id{0};
  std::vector<RigidBodyDescription> bones;
};

struct MarkerSetDescription
{
  std::string name;
  std::vector<std::string> marker_names;
};

struct DataDescriptions
{
  std::vector<MarkerSetDescription> marker_sets;
  std::vector<RigidBodyDescription> rigid_bodies;
  std::vector<SkeletonDescription> skeletons;
};

struct RigidBodyData
{
  int32_t id{0};
  std::array<float, 3> position{};
  std::array<float, 4> orientation{0.0f, 0.0f, 0.0f, 1.0f};  // x, y, z, w
  float mean_error{0.0f};
  int16_t params{0};

  bool tracking_valid() const {return (params & RB_TRACKING_VALID) != 0;}
};

struct SkeletonData
{
  int32_t id{0};
  std::vector<RigidBodyData> bones;
};

struct LabeledMarker
{
  int32_t id{0};  // asset id in the high word, member id in the low word
  std::array<float, 3> position{};
  float size{0.0f};
  int16_t params{0};
  float residual{0.0f};

  int32_t asset_id() const {return (id >> 16) & 0xffff;}
  int32_t member_id() const {return id & 0xffff;}
  bool occluded() const {return (params & MARKER_OCCLUDED) != 0;}
  bool unlabeled() const {return (params & MARKER_UNLABELED) != 0;}
};

struct FrameData
{
  int32_t frame_number{0};
  std::vector<std::array<float, 3>> unlabeled_markers;  // legacy "other markers"
  std::vector<RigidBodyData> rigid_bodies;
  std::vector<SkeletonData> skeletons;
  std::vector<LabeledMarker> labeled_markers;
  uint32_t timecode{0};
  uint32_t timecode_sub{0};
  double timestamp{0.0};
  uint64_t camera_mid_exposure{0};
  uint64_t camera_data_received{0};
  uint64_t transmit{0};
  int16_t params{0};

  bool models_changed() const {return (params & FRAME_MODELS_CHANGED) != 0;}
};

struct PacketHeader
{
  uint16_t id{0};
  uint16_t size{0};
};

// Parsing. data/size is the whole UDP payload, including the 4-byte header.
PacketHeader parse_header(const uint8_t * data, size_t size);
ServerInfo parse_server_info(const uint8_t * data, size_t size);
DataDescriptions parse_model_def(const uint8_t * data, size_t size, const Version & v);
FrameData parse_frame(const uint8_t * data, size_t size, const Version & v);

// Requests sent by the client.
std::vector<uint8_t> make_connect_request(const Version & requested);
std::vector<uint8_t> make_simple_request(uint16_t id);

// Encoding (used by tests and the fake server).
std::vector<uint8_t> encode_server_info(const ServerInfo & info);
std::vector<uint8_t> encode_model_def(const DataDescriptions & d, const Version & v);
std::vector<uint8_t> encode_frame(const FrameData & f, const Version & v);

}  // namespace natnet
}  // namespace mocap4r2_optitrack_driver

#endif  // MOCAP4R2_OPTITRACK_DRIVER__NATNET_PROTOCOL_HPP_
