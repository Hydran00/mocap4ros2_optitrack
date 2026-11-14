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

#include <gtest/gtest.h>

#include <cstddef>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "mocap4r2_optitrack_driver/natnet_protocol.hpp"

namespace natnet = mocap4r2_optitrack_driver::natnet;
using natnet::Version;

namespace
{

const std::vector<Version> kVersions = {{3, 0}, {3, 1}, {4, 0}, {4, 1}, {4, 2}, {4, 4}, {4, 5}};

natnet::DataDescriptions sample_descriptions()
{
  natnet::DataDescriptions d;
  natnet::MarkerSetDescription ms;
  ms.name = "robot";
  ms.marker_names = {"front", "left", "right"};
  d.marker_sets.push_back(ms);
  natnet::RigidBodyDescription rb;
  rb.name = "robot";
  rb.id = 3;
  rb.parent_id = -1;
  rb.offset = {0.1f, 0.2f, 0.3f};
  rb.orientation = {0.0f, 0.0f, 0.7071f, 0.7071f};
  rb.marker_names = {"front", "left", "right"};
  d.rigid_bodies.push_back(rb);
  natnet::SkeletonDescription sk;
  sk.name = "subject";
  sk.id = 7;
  for (int i = 1; i <= 3; i++) {
    natnet::RigidBodyDescription bone;
    bone.name = "bone" + std::to_string(i);
    bone.id = i;
    bone.parent_id = i - 1;
    bone.offset = {0.0f, 0.1f * i, 0.0f};
    bone.marker_names = {"a"};
    sk.bones.push_back(bone);
  }
  d.skeletons.push_back(sk);
  return d;
}

natnet::FrameData sample_frame()
{
  natnet::FrameData f;
  f.frame_number = 123456;
  f.unlabeled_markers = {{1.0f, 2.0f, 3.0f}};
  natnet::RigidBodyData rb;
  rb.id = 3;
  rb.position = {1.5f, -0.5f, 0.25f};
  rb.orientation = {0.1f, 0.2f, 0.3f, 0.9f};
  rb.mean_error = 0.0005f;
  rb.params = natnet::RB_TRACKING_VALID;
  f.rigid_bodies = {rb, rb};
  f.rigid_bodies[1].id = 4;
  f.rigid_bodies[1].params = 0;
  natnet::SkeletonData sk;
  sk.id = 7;
  for (int i = 1; i <= 3; i++) {
    natnet::RigidBodyData bone = rb;
    bone.id = (7 << 16) | i;
    sk.bones.push_back(bone);
  }
  f.skeletons = {sk};
  natnet::LabeledMarker lm;
  lm.id = (3 << 16) | 2;
  lm.position = {0.5f, 0.6f, 0.7f};
  lm.size = 0.014f;
  lm.params = natnet::MARKER_OCCLUDED;
  lm.residual = 0.0002f;
  f.labeled_markers = {lm};
  f.timecode = 11;
  f.timecode_sub = 22;
  f.timestamp = 12.5;
  f.camera_mid_exposure = 1000;
  f.camera_data_received = 1500;
  f.transmit = 2000;
  f.params = natnet::FRAME_MODELS_CHANGED;
  return f;
}

std::string name_of(const Version & v)
{
  return std::to_string(v.major) + "." + std::to_string(v.minor);
}

}  // namespace

TEST(NatNetProtocol, server_info_round_trip)
{
  natnet::ServerInfo in;
  in.app_name = "Motive";
  in.app_version = {3, 1, 2, 0};
  in.natnet_version = {4, 4, 0, 0};
  in.clock_frequency = 10000000;
  in.data_port = 1511;
  in.multicast = true;
  in.multicast_address = {239, 255, 42, 99};
  in.has_connection_info = true;
  const auto bytes = natnet::encode_server_info(in);
  const auto out = natnet::parse_server_info(bytes.data(), bytes.size());
  EXPECT_EQ(out.app_name, "Motive");
  EXPECT_EQ(out.natnet_version, in.natnet_version);
  EXPECT_EQ(out.app_version, in.app_version);
  EXPECT_EQ(out.clock_frequency, in.clock_frequency);
  EXPECT_EQ(out.data_port, 1511);
  EXPECT_TRUE(out.multicast);
  EXPECT_EQ(out.multicast_address, in.multicast_address);
  EXPECT_TRUE(out.has_connection_info);
  EXPECT_EQ(out.version().major, 4);
  EXPECT_EQ(out.version().minor, 4);
}

TEST(NatNetProtocol, server_info_without_connection_info)
{
  natnet::ServerInfo in;
  in.app_name = std::string(300, 'x');  // longer than the field: truncated
  in.natnet_version = {3, 0, 0, 0};
  const auto bytes = natnet::encode_server_info(in);
  const auto out = natnet::parse_server_info(bytes.data(), bytes.size());
  EXPECT_EQ(out.app_name.size(), 255u);
  EXPECT_FALSE(out.has_connection_info);
  EXPECT_EQ(out.data_port, 0);
}

TEST(NatNetProtocol, model_def_round_trip_all_versions)
{
  const auto in = sample_descriptions();
  for (const auto & v : kVersions) {
    SCOPED_TRACE(name_of(v));
    const auto bytes = natnet::encode_model_def(in, v);
    const auto out = natnet::parse_model_def(bytes.data(), bytes.size(), v);
    ASSERT_EQ(out.marker_sets.size(), 1u);
    EXPECT_EQ(out.marker_sets[0].marker_names, in.marker_sets[0].marker_names);
    ASSERT_EQ(out.rigid_bodies.size(), 1u);
    const auto & rb = out.rigid_bodies[0];
    EXPECT_EQ(rb.name, "robot");
    EXPECT_EQ(rb.id, 3);
    EXPECT_EQ(rb.parent_id, -1);
    EXPECT_FLOAT_EQ(rb.offset[2], 0.3f);
    if (v.at_least(4, 2)) {
      EXPECT_FLOAT_EQ(rb.orientation[2], 0.7071f);
    } else {
      EXPECT_FLOAT_EQ(rb.orientation[3], 1.0f);
    }
    ASSERT_EQ(rb.marker_names.size(), 3u);
    if (v.major >= 4) {
      EXPECT_EQ(rb.marker_names[1], "left");
    }
    ASSERT_EQ(out.skeletons.size(), 1u);
    EXPECT_EQ(out.skeletons[0].name, "subject");
    EXPECT_EQ(out.skeletons[0].id, 7);
    ASSERT_EQ(out.skeletons[0].bones.size(), 3u);
    EXPECT_EQ(out.skeletons[0].bones[2].name, "bone3");
    EXPECT_EQ(out.skeletons[0].bones[2].parent_id, 2);
  }
}

TEST(NatNetProtocol, frame_round_trip_all_versions)
{
  const auto in = sample_frame();
  for (const auto & v : kVersions) {
    SCOPED_TRACE(name_of(v));
    const auto bytes = natnet::encode_frame(in, v);
    const auto out = natnet::parse_frame(bytes.data(), bytes.size(), v);
    EXPECT_EQ(out.frame_number, 123456);
    ASSERT_EQ(out.unlabeled_markers.size(), 1u);
    EXPECT_FLOAT_EQ(out.unlabeled_markers[0][2], 3.0f);
    ASSERT_EQ(out.rigid_bodies.size(), 2u);
    EXPECT_EQ(out.rigid_bodies[0].id, 3);
    EXPECT_FLOAT_EQ(out.rigid_bodies[0].position[0], 1.5f);
    EXPECT_FLOAT_EQ(out.rigid_bodies[0].orientation[3], 0.9f);
    EXPECT_FLOAT_EQ(out.rigid_bodies[0].mean_error, 0.0005f);
    EXPECT_TRUE(out.rigid_bodies[0].tracking_valid());
    EXPECT_FALSE(out.rigid_bodies[1].tracking_valid());
    ASSERT_EQ(out.skeletons.size(), 1u);
    ASSERT_EQ(out.skeletons[0].bones.size(), 3u);
    EXPECT_EQ(out.skeletons[0].bones[1].id & 0xffff, 2);
    ASSERT_EQ(out.labeled_markers.size(), 1u);
    const auto & lm = out.labeled_markers[0];
    EXPECT_EQ(lm.asset_id(), 3);
    EXPECT_EQ(lm.member_id(), 2);
    EXPECT_TRUE(lm.occluded());
    EXPECT_FALSE(lm.unlabeled());
    EXPECT_FLOAT_EQ(lm.residual, 0.0002f);
    EXPECT_EQ(out.timecode, 11u);
    EXPECT_DOUBLE_EQ(out.timestamp, 12.5);
    EXPECT_EQ(out.camera_mid_exposure, 1000u);
    EXPECT_EQ(out.transmit, 2000u);
    EXPECT_TRUE(out.models_changed());
  }
}

TEST(NatNetProtocol, empty_frame)
{
  natnet::FrameData in;
  in.frame_number = 1;
  for (const auto & v : kVersions) {
    const auto bytes = natnet::encode_frame(in, v);
    const auto out = natnet::parse_frame(bytes.data(), bytes.size(), v);
    EXPECT_TRUE(out.rigid_bodies.empty());
    EXPECT_TRUE(out.labeled_markers.empty());
  }
}

TEST(NatNetProtocol, every_truncation_is_a_parse_error)
{
  const auto frame = sample_frame();
  const auto desc = sample_descriptions();
  for (const auto & v : kVersions) {
    SCOPED_TRACE(name_of(v));
    const auto fb = natnet::encode_frame(frame, v);
    // The last 4 bytes are the end-of-data tag, which is not needed.
    for (size_t n = 0; n + 4 < fb.size(); n++) {
      std::vector<uint8_t> cut(fb.begin(), fb.begin() + n);
      if (n >= 4) {
        const uint16_t sz = static_cast<uint16_t>(n - 4);
        std::memcpy(&cut[2], &sz, 2);
      }
      EXPECT_THROW(natnet::parse_frame(cut.data(), cut.size(), v), natnet::ParseError) << n;
    }
    const auto db = natnet::encode_model_def(desc, v);
    for (size_t n = 0; n < db.size(); n++) {
      std::vector<uint8_t> cut(db.begin(), db.begin() + n);
      EXPECT_THROW(natnet::parse_model_def(cut.data(), cut.size(), v), natnet::ParseError) << n;
    }
  }
}

TEST(NatNetProtocol, header_size_bounds_the_payload)
{
  // A header announcing fewer bytes than the datagram must not let the parser
  // read the rest; one announcing more is clipped to the datagram.
  const auto bytes = natnet::encode_frame(sample_frame(), {4, 4});
  auto small = bytes;
  const uint16_t sz = 10;
  std::memcpy(&small[2], &sz, 2);
  EXPECT_THROW(natnet::parse_frame(small.data(), small.size(), {4, 4}), natnet::ParseError);
  auto big = bytes;
  const uint16_t huge = 0xffff;
  std::memcpy(&big[2], &huge, 2);
  EXPECT_NO_THROW(natnet::parse_frame(big.data(), big.size(), {4, 4}));
}

TEST(NatNetProtocol, wrong_message_and_invalid_counts)
{
  const auto info = natnet::encode_server_info(natnet::ServerInfo{});
  EXPECT_THROW(natnet::parse_frame(info.data(), info.size(), {4, 4}), natnet::ParseError);
  EXPECT_THROW(natnet::parse_header(info.data(), 3), natnet::ParseError);

  auto bytes = natnet::encode_frame(natnet::FrameData{}, {3, 0});
  // Marker set count right after the frame number: negative, then huge.
  for (int32_t bad : {-1, 0x7fffffff}) {
    auto b = bytes;
    std::memcpy(&b[8], &bad, 4);
    EXPECT_THROW(natnet::parse_frame(b.data(), b.size(), {3, 0}), natnet::ParseError);
  }
  // Negative block size (4.1+).
  auto b41 = natnet::encode_frame(natnet::FrameData{}, {4, 1});
  const int32_t neg = -5;
  std::memcpy(&b41[12], &neg, 4);
  EXPECT_THROW(natnet::parse_frame(b41.data(), b41.size(), {4, 1}), natnet::ParseError);
}

TEST(NatNetProtocol, unknown_description_types)
{
  // Build a 4.4 model def with an unknown dataset type (e.g. a camera) in front.
  const auto known = natnet::encode_model_def(sample_descriptions(), {4, 4});
  std::vector<uint8_t> payload(known.begin() + 4, known.end());
  int32_t count;
  std::memcpy(&count, payload.data(), 4);
  count++;
  std::memcpy(payload.data(), &count, 4);
  std::vector<uint8_t> extra = {5, 0, 0, 0, 3, 0, 0, 0, 'a', 'b', 'c'};  // type 5, 3 bytes
  payload.insert(payload.begin() + 4, extra.begin(), extra.end());
  std::vector<uint8_t> pkt = {natnet::MSG_MODELDEF, 0, 0, 0};
  const uint16_t sz = static_cast<uint16_t>(payload.size());
  std::memcpy(&pkt[2], &sz, 2);
  pkt.insert(pkt.end(), payload.begin(), payload.end());
  const auto d = natnet::parse_model_def(pkt.data(), pkt.size(), {4, 4});
  EXPECT_EQ(d.rigid_bodies.size(), 1u);
  EXPECT_EQ(d.skeletons.size(), 1u);
  // Without block sizes an unknown type cannot be skipped.
  std::vector<uint8_t> old = {natnet::MSG_MODELDEF, 0, 8, 0, 1, 0, 0, 0, 5, 0, 0, 0};
  EXPECT_THROW(natnet::parse_model_def(old.data(), old.size(), {4, 0}), natnet::ParseError);
}

TEST(NatNetProtocol, newer_fields_inside_blocks_are_skipped)
{
  // A future server adds 4 bytes at the end of the rigid body block.
  natnet::FrameData in;
  in.frame_number = 9;
  natnet::RigidBodyData rb;
  rb.id = 1;
  rb.params = natnet::RB_TRACKING_VALID;
  in.rigid_bodies = {rb};
  natnet::LabeledMarker lm;
  lm.id = (1 << 16) | 1;
  in.labeled_markers = {lm};
  const Version v{4, 4};
  auto bytes = natnet::encode_frame(in, v);
  // Layout: hdr(4) frame(4) sets[count,size](8) other[count,size](8) rb[count,size](8) rb data
  const size_t rb_size_at = 4 + 4 + 8 + 8 + 4;
  int32_t size;
  std::memcpy(&size, &bytes[rb_size_at], 4);
  const size_t insert_at = rb_size_at + 4 + static_cast<size_t>(size);
  const std::vector<uint8_t> extra = {0xde, 0xad, 0xbe, 0xef};
  bytes.insert(bytes.begin() + static_cast<std::ptrdiff_t>(insert_at), extra.begin(), extra.end());
  size += 4;
  std::memcpy(&bytes[rb_size_at], &size, 4);
  const uint16_t total = static_cast<uint16_t>(bytes.size() - 4);
  std::memcpy(&bytes[2], &total, 2);
  const auto out = natnet::parse_frame(bytes.data(), bytes.size(), v);
  ASSERT_EQ(out.rigid_bodies.size(), 1u);
  ASSERT_EQ(out.labeled_markers.size(), 1u);
  EXPECT_EQ(out.labeled_markers[0].member_id(), 1);
}

TEST(NatNetProtocol, random_corruption_never_crashes)
{
  std::mt19937 rng(1234);
  const auto frame = sample_frame();
  for (const auto & v : kVersions) {
    const auto base = natnet::encode_frame(frame, v);
    std::uniform_int_distribution<size_t> pos(4, base.size() - 1);
    std::uniform_int_distribution<int> byte(0, 255);
    for (int i = 0; i < 3000; i++) {
      auto b = base;
      const int changes = 1 + i % 4;
      for (int k = 0; k < changes; k++) {
        b[pos(rng)] = static_cast<uint8_t>(byte(rng));
      }
      try {
        natnet::parse_frame(b.data(), b.size(), v);
      } catch (const natnet::ParseError &) {
      }
    }
  }
  SUCCEED();
}

TEST(NatNetProtocol, future_bitstream_with_unknown_blocks)
{
  // A 4.6 server adds a new size-prefixed block before the frame suffix:
  // encode as 4.5, insert the block, and parse as 4.6.
  const auto in = sample_frame();
  auto bytes = natnet::encode_frame(in, {4, 5});
  const size_t suffix = 4 + 4 + 8 + 3 * 8 + 4 + 4 + 2 + 4;  // + end tag
  const std::vector<uint8_t> block = {1, 0, 0, 0, 4, 0, 0, 0, 9, 9, 9, 9};
  bytes.insert(bytes.end() - static_cast<std::ptrdiff_t>(suffix), block.begin(), block.end());
  const uint16_t total = static_cast<uint16_t>(bytes.size() - 4);
  std::memcpy(&bytes[2], &total, 2);
  const auto out = natnet::parse_frame(bytes.data(), bytes.size(), {4, 6});
  EXPECT_EQ(out.rigid_bodies.size(), 2u);
  EXPECT_EQ(out.labeled_markers.size(), 1u);
  EXPECT_EQ(out.camera_mid_exposure, 1000u);
  EXPECT_EQ(out.transmit, 2000u);
  EXPECT_TRUE(out.models_changed());
}

TEST(NatNetProtocol, requests)
{
  const auto c = natnet::make_connect_request({4, 4});
  const auto h = natnet::parse_header(c.data(), c.size());
  EXPECT_EQ(h.id, natnet::MSG_CONNECT);
  EXPECT_EQ(h.size + 4u, c.size());
  EXPECT_EQ(std::string(reinterpret_cast<const char *>(&c[4]), 4), "Ping");
  EXPECT_EQ(c[4 + 265], 4);
  EXPECT_EQ(c[4 + 266], 4);
  const auto k = natnet::make_simple_request(natnet::MSG_KEEPALIVE);
  EXPECT_EQ(k.size(), 4u);
  EXPECT_EQ(natnet::parse_header(k.data(), k.size()).id, natnet::MSG_KEEPALIVE);
}
