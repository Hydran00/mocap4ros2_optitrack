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

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

#include "mocap4r2_optitrack_driver/natnet_protocol.hpp"

namespace mocap4r2_optitrack_driver
{
namespace natnet
{

namespace
{

constexpr size_t kNameLength = 256;

// Bounds-checked little-endian reader.
class Reader
{
public:
  Reader(const uint8_t * data, size_t size)
  : p_(data), end_(data + size) {}

  template<typename T>
  T get()
  {
    need(sizeof(T));
    T v;
    std::memcpy(&v, p_, sizeof(T));
    p_ += sizeof(T);
    return v;
  }

  std::array<float, 3> vec3() {return {get<float>(), get<float>(), get<float>()};}
  std::array<float, 4> quat() {return {get<float>(), get<float>(), get<float>(), get<float>()};}

  std::string cstr()
  {
    const auto * nul = static_cast<const uint8_t *>(std::memchr(p_, 0, remaining()));
    if (!nul) {
      throw ParseError("unterminated string");
    }
    std::string s(reinterpret_cast<const char *>(p_), nul - p_);
    p_ = nul + 1;
    return s;
  }

  // Element count: non negative and plausible for the bytes left.
  size_t count(size_t min_element_size)
  {
    const int32_t n = get<int32_t>();
    if (n < 0 || static_cast<size_t>(n) * std::max<size_t>(min_element_size, 1) > remaining()) {
      throw ParseError("invalid element count " + std::to_string(n));
    }
    return static_cast<size_t>(n);
  }

  void skip(size_t n)
  {
    need(n);
    p_ += n;
  }

  // Reader over the next n bytes; this reader jumps past them.
  Reader sub(size_t n)
  {
    need(n);
    Reader r(p_, n);
    p_ += n;
    return r;
  }

  size_t remaining() const {return static_cast<size_t>(end_ - p_);}
  const uint8_t * pos() const {return p_;}

private:
  void need(size_t n) const
  {
    if (n > remaining()) {
      throw ParseError("truncated packet");
    }
  }

  const uint8_t * p_;
  const uint8_t * end_;
};

class Writer
{
public:
  template<typename T>
  void put(const T & v)
  {
    const auto * b = reinterpret_cast<const uint8_t *>(&v);
    buf_.insert(buf_.end(), b, b + sizeof(T));
  }
  void vec3(const std::array<float, 3> & v) {for (float f : v) {put(f);}}
  void quat(const std::array<float, 4> & v) {for (float f : v) {put(f);}}
  void cstr(const std::string & s)
  {
    buf_.insert(buf_.end(), s.begin(), s.end());
    buf_.push_back(0);
  }
  void bytes(const std::vector<uint8_t> & b) {buf_.insert(buf_.end(), b.begin(), b.end());}
  size_t size() const {return buf_.size();}
  std::vector<uint8_t> & buffer() {return buf_;}

  // Write a count, and a size placeholder from 4.1 that end_block fills.
  size_t begin_block(int32_t count, const Version & v)
  {
    put(count);
    if (!v.has_block_sizes()) {
      return 0;
    }
    put(int32_t{0});
    return buf_.size();
  }
  void end_block(size_t start, const Version & v)
  {
    if (!v.has_block_sizes()) {
      return;
    }
    const int32_t n = static_cast<int32_t>(buf_.size() - start);
    std::memcpy(&buf_[start - 4], &n, 4);
  }

private:
  std::vector<uint8_t> buf_;
};

std::vector<uint8_t> packet(uint16_t id, const std::vector<uint8_t> & payload)
{
  if (payload.size() > 0xffff) {
    throw std::length_error("NatNet payload too large");
  }
  Writer w;
  w.put(id);
  w.put(static_cast<uint16_t>(payload.size()));
  w.bytes(payload);
  return w.buffer();
}

// Payload of a packet of the expected type, limited to the size in its header.
Reader payload_of(const uint8_t * data, size_t size, uint16_t expected)
{
  const auto h = parse_header(data, size);
  if (h.id != expected) {
    throw ParseError("unexpected message id " + std::to_string(h.id));
  }
  // Some servers send a size smaller than the datagram; never read past either.
  const size_t n = std::min<size_t>(h.size, size - 4);
  return Reader(data + 4, n);
}

// For blocks with a size prefix (4.1+): run parse on the block and skip what it
// did not consume, so that data added by newer servers is ignored.
template<typename F>
void block(Reader & r, const Version & v, F && parse)
{
  if (v.has_block_sizes()) {
    const int32_t n = r.get<int32_t>();
    if (n < 0) {
      throw ParseError("negative block size");
    }
    Reader b = r.sub(static_cast<size_t>(n));
    parse(b);
  } else {
    parse(r);
  }
}

RigidBodyDescription read_rb_description(Reader & r, const Version & v)
{
  RigidBodyDescription d;
  if (v.major >= 2) {
    d.name = r.cstr();
  }
  d.id = r.get<int32_t>();
  d.parent_id = r.get<int32_t>();
  d.offset = r.vec3();
  if (v.at_least(4, 2)) {
    d.orientation = r.quat();
  }
  if (v.major >= 3) {
    const size_t n = r.count(16);
    r.skip(n * 12);  // marker offsets
    r.skip(n * 4);   // active labels
    if (v.major >= 4) {
      for (size_t i = 0; i < n; i++) {
        d.marker_names.push_back(r.cstr());
      }
    } else {
      d.marker_names.resize(n);
    }
  }
  return d;
}

void write_rb_description(Writer & w, const RigidBodyDescription & d, const Version & v)
{
  if (v.major >= 2) {
    w.cstr(d.name);
  }
  w.put(d.id);
  w.put(d.parent_id);
  w.vec3(d.offset);
  if (v.at_least(4, 2)) {
    w.quat(d.orientation);
  }
  if (v.major >= 3) {
    const auto n = static_cast<int32_t>(d.marker_names.size());
    w.put(n);
    for (int32_t i = 0; i < n; i++) {
      w.vec3({0.0f, 0.0f, 0.0f});
    }
    for (int32_t i = 0; i < n; i++) {
      w.put(int32_t{0});
    }
    if (v.major >= 4) {
      for (const auto & name : d.marker_names) {
        w.cstr(name);
      }
    }
  }
}

RigidBodyData read_rb_data(Reader & r, const Version & v)
{
  RigidBodyData rb;
  rb.id = r.get<int32_t>();
  rb.position = r.vec3();
  rb.orientation = r.quat();
  if (v.major >= 2) {
    rb.mean_error = r.get<float>();
  }
  if (v.at_least(2, 6)) {
    rb.params = r.get<int16_t>();
  } else {
    rb.params = RB_TRACKING_VALID;
  }
  return rb;
}

void write_rb_data(Writer & w, const RigidBodyData & rb, const Version & v)
{
  w.put(rb.id);
  w.vec3(rb.position);
  w.quat(rb.orientation);
  if (v.major >= 2) {
    w.put(rb.mean_error);
  }
  if (v.at_least(2, 6)) {
    w.put(rb.params);
  }
}

// Force plates and devices share a layout: id, channels, frames of floats.
void skip_analog(Reader & r, size_t n)
{
  for (size_t i = 0; i < n; i++) {
    r.get<int32_t>();
    const size_t channels = r.count(4);
    for (size_t c = 0; c < channels; c++) {
      const size_t frames = r.count(4);
      r.skip(frames * 4);
    }
  }
}

}  // namespace

PacketHeader
parse_header(const uint8_t * data, size_t size)
{
  Reader r(data, size);
  PacketHeader h;
  h.id = r.get<uint16_t>();
  h.size = r.get<uint16_t>();
  return h;
}

ServerInfo
parse_server_info(const uint8_t * data, size_t size)
{
  Reader r = payload_of(data, size, MSG_SERVERINFO);
  ServerInfo info;
  Reader name = r.sub(kNameLength);
  const auto * p = name.pos();
  info.app_name.assign(
    reinterpret_cast<const char *>(p),
    strnlen(reinterpret_cast<const char *>(p), kNameLength));
  for (auto & b : info.app_version) {
    b = r.get<uint8_t>();
  }
  for (auto & b : info.natnet_version) {
    b = r.get<uint8_t>();
  }
  // Connection info (Motive 3 and later)
  if (r.remaining() >= 8 + 2 + 1 + 4) {
    info.clock_frequency = r.get<uint64_t>();
    info.data_port = r.get<uint16_t>();
    info.multicast = r.get<uint8_t>() != 0;
    for (auto & b : info.multicast_address) {
      b = r.get<uint8_t>();
    }
    info.has_connection_info = true;
  } else if (r.remaining() >= 8) {
    info.clock_frequency = r.get<uint64_t>();
  }
  return info;
}

DataDescriptions
parse_model_def(const uint8_t * data, size_t size, const Version & v)
{
  Reader r = payload_of(data, size, MSG_MODELDEF);
  DataDescriptions d;
  const size_t n = r.count(4);
  for (size_t i = 0; i < n; i++) {
    const int32_t type = r.get<int32_t>();
    auto parse_one = [&](Reader & b) {
        switch (type) {
          case DESC_MARKERSET: {
              MarkerSetDescription ms;
              ms.name = b.cstr();
              const size_t m = b.count(1);
              for (size_t k = 0; k < m; k++) {
                ms.marker_names.push_back(b.cstr());
              }
              d.marker_sets.push_back(ms);
              break;
            }
          case DESC_RIGIDBODY:
            d.rigid_bodies.push_back(read_rb_description(b, v));
            break;
          case DESC_SKELETON: {
              SkeletonDescription sk;
              sk.name = b.cstr();
              sk.id = b.get<int32_t>();
              const size_t m = b.count(16);
              for (size_t k = 0; k < m; k++) {
                sk.bones.push_back(read_rb_description(b, v));
              }
              d.skeletons.push_back(sk);
              break;
            }
          default:
            // Other types (force plates, devices, cameras, assets) can only be
            // skipped when the server sends block sizes.
            if (!v.has_block_sizes()) {
              throw ParseError("unsupported description type " + std::to_string(type));
            }
        }
      };
    block(r, v, parse_one);
  }
  return d;
}

FrameData
parse_frame(const uint8_t * data, size_t size, const Version & v)
{
  Reader r = payload_of(data, size, MSG_FRAMEOFDATA);
  FrameData f;
  f.frame_number = r.get<int32_t>();

  // Marker sets: positions are repeated in the labeled markers, so skipped.
  const size_t n_sets = r.count(5);
  block(
    r, v, [&](Reader & b) {
      for (size_t i = 0; i < n_sets; i++) {
        b.cstr();
        const size_t m = b.count(12);
        b.skip(m * 12);
      }
    });

  const size_t n_other = r.count(12);
  block(
    r, v, [&](Reader & b) {
      for (size_t i = 0; i < n_other; i++) {
        f.unlabeled_markers.push_back(b.vec3());
      }
    });

  const size_t n_rb = r.count(32);
  block(
    r, v, [&](Reader & b) {
      for (size_t i = 0; i < n_rb; i++) {
        f.rigid_bodies.push_back(read_rb_data(b, v));
      }
    });

  if (v.at_least(2, 1)) {
    const size_t n_sk = r.count(8);
    block(
      r, v, [&](Reader & b) {
        for (size_t i = 0; i < n_sk; i++) {
          SkeletonData sk;
          sk.id = b.get<int32_t>();
          const size_t m = b.count(32);
          for (size_t k = 0; k < m; k++) {
            sk.bones.push_back(read_rb_data(b, v));
          }
          f.skeletons.push_back(sk);
        }
      });
  }

  if (v.has_block_sizes()) {
    r.count(0);  // assets: skipped through the block size
    block(r, v, [](Reader &) {});
  }

  if (v.at_least(2, 3)) {
    const size_t n_lm = r.count(20);
    block(
      r, v, [&](Reader & b) {
        for (size_t i = 0; i < n_lm; i++) {
          LabeledMarker m;
          m.id = b.get<int32_t>();
          m.position = b.vec3();
          m.size = b.get<float>();
          if (v.at_least(2, 6)) {
            m.params = b.get<int16_t>();
          }
          if (v.major >= 3) {
            m.residual = b.get<float>();
          }
          f.labeled_markers.push_back(m);
        }
      });
  }

  if (v.at_least(2, 9)) {
    const size_t n_plates = r.count(8);
    block(r, v, [&](Reader & b) {skip_analog(b, n_plates);});
  }
  if (v.at_least(2, 11)) {
    const size_t n_devices = r.count(8);
    block(r, v, [&](Reader & b) {skip_analog(b, n_devices);});
  }

  // NatNet 4.5: IMU and GPIO blocks (size-prefixed, not decoded).
  if (v.at_least(4, 5)) {
    r.count(0);
    block(r, v, [](Reader &) {});
    r.count(0);
    block(r, v, [](Reader &) {});
  }
  // Bitstreams newer than this parser may add blocks before the suffix, which
  // has a fixed size: read it from the end of the packet (before the end tag).
  if (v.major > 4 || (v.major == 4 && v.minor > 5)) {
    constexpr size_t kSuffix = 4 + 4 + 8 + 3 * 8 + 4 + 4 + 2;
    if (r.remaining() < kSuffix + 4) {
      throw ParseError("truncated packet");
    }
    r.skip(r.remaining() - kSuffix - 4);
  }

  if (v.major < 3) {
    r.get<float>();  // software latency
  }
  f.timecode = r.get<uint32_t>();
  f.timecode_sub = r.get<uint32_t>();
  if (v.at_least(2, 7)) {
    f.timestamp = r.get<double>();
  } else {
    f.timestamp = r.get<float>();
  }
  if (v.major >= 3) {
    f.camera_mid_exposure = r.get<uint64_t>();
    f.camera_data_received = r.get<uint64_t>();
    f.transmit = r.get<uint64_t>();
  }
  if (v.has_block_sizes()) {
    r.get<uint32_t>();  // precision timestamp, seconds
    r.get<uint32_t>();  // precision timestamp, fraction
  }
  f.params = r.get<int16_t>();
  return f;
}

std::vector<uint8_t>
make_connect_request(const Version & requested)
{
  // "Ping", padded, then the bitstream version the client wants (unicast).
  std::vector<uint8_t> payload(270, 0);
  const char ping[] = "Ping";
  std::memcpy(payload.data(), ping, 4);
  payload[265] = static_cast<uint8_t>(requested.major);
  payload[266] = static_cast<uint8_t>(requested.minor);
  payload.push_back(0);
  return packet(MSG_CONNECT, payload);
}

std::vector<uint8_t>
make_simple_request(uint16_t id)
{
  return packet(id, {});
}

std::vector<uint8_t>
encode_server_info(const ServerInfo & info)
{
  Writer w;
  std::vector<uint8_t> name(kNameLength, 0);
  std::memcpy(name.data(), info.app_name.data(), std::min(info.app_name.size(), kNameLength - 1));
  w.bytes(name);
  for (auto b : info.app_version) {
    w.put(b);
  }
  for (auto b : info.natnet_version) {
    w.put(b);
  }
  if (info.has_connection_info) {
    w.put(info.clock_frequency);
    w.put(info.data_port);
    w.put(static_cast<uint8_t>(info.multicast ? 1 : 0));
    for (auto b : info.multicast_address) {
      w.put(b);
    }
  }
  return packet(MSG_SERVERINFO, w.buffer());
}

std::vector<uint8_t>
encode_model_def(const DataDescriptions & d, const Version & v)
{
  Writer w;
  w.put(static_cast<int32_t>(d.marker_sets.size() + d.rigid_bodies.size() + d.skeletons.size()));
  auto dataset = [&](int32_t type, auto && body) {
      w.put(type);
      size_t start = 0;
      if (v.has_block_sizes()) {
        w.put(int32_t{0});
        start = w.size();
      }
      body();
      if (v.has_block_sizes()) {
        const int32_t n = static_cast<int32_t>(w.size() - start);
        std::memcpy(&w.buffer()[start - 4], &n, 4);
      }
    };
  for (const auto & ms : d.marker_sets) {
    dataset(
      DESC_MARKERSET, [&] {
        w.cstr(ms.name);
        w.put(static_cast<int32_t>(ms.marker_names.size()));
        for (const auto & n : ms.marker_names) {
          w.cstr(n);
        }
      });
  }
  for (const auto & rb : d.rigid_bodies) {
    dataset(DESC_RIGIDBODY, [&] {write_rb_description(w, rb, v);});
  }
  for (const auto & sk : d.skeletons) {
    dataset(
      DESC_SKELETON, [&] {
        w.cstr(sk.name);
        w.put(sk.id);
        w.put(static_cast<int32_t>(sk.bones.size()));
        for (const auto & b : sk.bones) {
          write_rb_description(w, b, v);
        }
      });
  }
  return packet(MSG_MODELDEF, w.buffer());
}

std::vector<uint8_t>
encode_frame(const FrameData & f, const Version & v)
{
  Writer w;
  w.put(f.frame_number);

  auto start = w.begin_block(0, v);  // no marker sets
  w.end_block(start, v);

  start = w.begin_block(static_cast<int32_t>(f.unlabeled_markers.size()), v);
  for (const auto & m : f.unlabeled_markers) {
    w.vec3(m);
  }
  w.end_block(start, v);

  start = w.begin_block(static_cast<int32_t>(f.rigid_bodies.size()), v);
  for (const auto & rb : f.rigid_bodies) {
    write_rb_data(w, rb, v);
  }
  w.end_block(start, v);

  if (v.at_least(2, 1)) {
    start = w.begin_block(static_cast<int32_t>(f.skeletons.size()), v);
    for (const auto & sk : f.skeletons) {
      w.put(sk.id);
      w.put(static_cast<int32_t>(sk.bones.size()));
      for (const auto & b : sk.bones) {
        write_rb_data(w, b, v);
      }
    }
    w.end_block(start, v);
  }

  if (v.has_block_sizes()) {
    start = w.begin_block(0, v);  // no assets
    w.end_block(start, v);
  }

  if (v.at_least(2, 3)) {
    start = w.begin_block(static_cast<int32_t>(f.labeled_markers.size()), v);
    for (const auto & m : f.labeled_markers) {
      w.put(m.id);
      w.vec3(m.position);
      w.put(m.size);
      if (v.at_least(2, 6)) {
        w.put(m.params);
      }
      if (v.major >= 3) {
        w.put(m.residual);
      }
    }
    w.end_block(start, v);
  }

  if (v.at_least(2, 9)) {
    start = w.begin_block(0, v);  // no force plates
    w.end_block(start, v);
  }
  if (v.at_least(2, 11)) {
    start = w.begin_block(0, v);  // no devices
    w.end_block(start, v);
  }
  if (v.at_least(4, 5)) {
    start = w.begin_block(0, v);  // no IMUs
    w.end_block(start, v);
    start = w.begin_block(0, v);  // no GPIO
    w.end_block(start, v);
  }

  if (v.major < 3) {
    w.put(0.0f);
  }
  w.put(f.timecode);
  w.put(f.timecode_sub);
  if (v.at_least(2, 7)) {
    w.put(f.timestamp);
  } else {
    w.put(static_cast<float>(f.timestamp));
  }
  if (v.major >= 3) {
    w.put(f.camera_mid_exposure);
    w.put(f.camera_data_received);
    w.put(f.transmit);
  }
  if (v.has_block_sizes()) {
    w.put(uint32_t{0});
    w.put(uint32_t{0});
  }
  w.put(f.params);
  w.put(int32_t{0});  // end of data
  return packet(MSG_FRAMEOFDATA, w.buffer());
}

}  // namespace natnet
}  // namespace mocap4r2_optitrack_driver
