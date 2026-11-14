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

#include <cmath>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "mocap4r2_optitrack_driver/frame_converter.hpp"

namespace mocap4r2_optitrack_driver
{

namespace
{

using mocap4r2_interfaces::msg::Marker;
using mocap4r2_interfaces::msg::RigidBody;
using mocap4r2_interfaces::msg::Segment;
using mocap4r2_interfaces::msg::Skeleton;

struct Pose
{
  double p[3]{0.0, 0.0, 0.0};
  double q[4]{0.0, 0.0, 0.0, 1.0};  // x, y, z, w
};

// Y-up (Motive default) to Z-up: +90 degrees about X, (x, y, z) -> (x, -z, y).
// Applied to positions and, by conjugation, to orientations.
void to_z_up(double v[3])
{
  const double y = v[1];
  v[1] = -v[2];
  v[2] = y;
}

Pose make_pose(const std::array<float, 3> & p, const std::array<float, 4> & q, UpAxis up)
{
  Pose out;
  for (int i = 0; i < 3; i++) {
    out.p[i] = p[i];
  }
  for (int i = 0; i < 4; i++) {
    out.q[i] = q[i];
  }
  const double n = std::sqrt(
    out.q[0] * out.q[0] + out.q[1] * out.q[1] + out.q[2] * out.q[2] + out.q[3] * out.q[3]);
  if (n > 1e-9) {
    for (double & c : out.q) {
      c /= n;
    }
  } else {
    out.q[0] = out.q[1] = out.q[2] = 0.0;
    out.q[3] = 1.0;
  }
  if (up == UpAxis::Y) {
    to_z_up(out.p);
    to_z_up(out.q);  // vector part rotates like a position
  }
  return out;
}

Pose compose(const Pose & a, const Pose & b)
{
  const double ax = a.q[0], ay = a.q[1], az = a.q[2], aw = a.q[3];
  const double bx = b.q[0], by = b.q[1], bz = b.q[2], bw = b.q[3];
  Pose r;
  r.q[0] = aw * bx + ax * bw + ay * bz - az * by;
  r.q[1] = aw * by - ax * bz + ay * bw + az * bx;
  r.q[2] = aw * bz + ax * by - ay * bx + az * bw;
  r.q[3] = aw * bw - ax * bx - ay * by - az * bz;
  // rotate b.p by a.q: p + 2w(u x p) + 2u x (u x p)
  const double px = b.p[0], py = b.p[1], pz = b.p[2];
  const double cx = ay * pz - az * py, cy = az * px - ax * pz, cz = ax * py - ay * px;
  r.p[0] = a.p[0] + px + 2.0 * (aw * cx + ay * cz - az * cy);
  r.p[1] = a.p[1] + py + 2.0 * (aw * cy + az * cx - ax * cz);
  r.p[2] = a.p[2] + pz + 2.0 * (aw * cz + ax * cy - ay * cx);
  return r;
}

geometry_msgs::msg::Pose to_msg(const Pose & p)
{
  geometry_msgs::msg::Pose m;
  m.position.x = p.p[0];
  m.position.y = p.p[1];
  m.position.z = p.p[2];
  m.orientation.x = p.q[0];
  m.orientation.y = p.q[1];
  m.orientation.z = p.q[2];
  m.orientation.w = p.q[3];
  return m;
}

}  // namespace

Model
Model::from(const natnet::DataDescriptions & d)
{
  Model m;
  for (const auto & rb : d.rigid_bodies) {
    m.asset_names[rb.id] = rb.name;
  }
  for (const auto & sk : d.skeletons) {
    m.asset_names[sk.id] = sk.name;
    m.skeletons[sk.id] = sk;
  }
  for (const auto & ms : d.marker_sets) {
    m.marker_names[ms.name] = ms.marker_names;
  }
  // NatNet 4 rigid body descriptions carry their own marker names.
  for (const auto & rb : d.rigid_bodies) {
    if (!rb.marker_names.empty() && !rb.marker_names[0].empty()) {
      m.marker_names[rb.name] = rb.marker_names;
    }
  }
  return m;
}

double
frame_latency(const natnet::FrameData & frame, uint64_t clock_frequency)
{
  if (clock_frequency == 0 || frame.camera_mid_exposure == 0 ||
    frame.transmit < frame.camera_mid_exposure)
  {
    return 0.0;
  }
  return static_cast<double>(frame.transmit - frame.camera_mid_exposure) /
         static_cast<double>(clock_frequency);
}

mocap4r2_control::MocapFrame
convert_frame(
  const natnet::FrameData & frame, const Model & model, const ConverterOptions & options)
{
  mocap4r2_control::MocapFrame out;
  out.frame_number = static_cast<uint64_t>(static_cast<uint32_t>(frame.frame_number));

  // Labeled markers (includes unlabeled point-cloud markers, flagged).
  std::map<int32_t, std::vector<Marker>> by_asset;
  for (const auto & lm : frame.labeled_markers) {
    Marker m;
    const Pose p = make_pose(lm.position, {0.0f, 0.0f, 0.0f, 1.0f}, options.up_axis);
    m.position.x = p.p[0];
    m.position.y = p.p[1];
    m.position.z = p.p[2];
    m.occluded = lm.occluded();
    m.residual = lm.residual;
    if (lm.unlabeled() || lm.asset_id() == 0) {
      m.id = Marker::UNKNOWN_ID;
    } else {
      m.id = lm.id;
      const auto asset = model.asset_names.find(lm.asset_id());
      if (asset != model.asset_names.end()) {
        const auto names = model.marker_names.find(asset->second);
        const int idx = lm.member_id() - 1;
        if (names != model.marker_names.end() && idx >= 0 &&
          idx < static_cast<int>(names->second.size()) && !names->second[idx].empty())
        {
          m.name = asset->second + ":" + names->second[idx];
        } else {
          m.name = asset->second + ":" + std::to_string(lm.member_id());
        }
      }
      by_asset[lm.asset_id()].push_back(m);
    }
    out.markers.push_back(m);
  }
  // Legacy unlabeled markers (empty in recent Motive versions).
  for (const auto & um : frame.unlabeled_markers) {
    Marker m;
    const Pose p = make_pose(um, {0.0f, 0.0f, 0.0f, 1.0f}, options.up_axis);
    m.id = Marker::UNKNOWN_ID;
    m.position.x = p.p[0];
    m.position.y = p.p[1];
    m.position.z = p.p[2];
    m.residual = -1.0f;
    out.markers.push_back(m);
  }

  for (const auto & rb : frame.rigid_bodies) {
    RigidBody body;
    body.id = rb.id;
    const auto name = model.asset_names.find(rb.id);
    body.name = name != model.asset_names.end() ? name->second : std::to_string(rb.id);
    body.tracking_valid = rb.tracking_valid();
    body.pose = to_msg(make_pose(rb.position, rb.orientation, options.up_axis));
    body.mean_error = rb.mean_error;
    const auto markers = by_asset.find(rb.id);
    if (markers != by_asset.end()) {
      body.markers = markers->second;
    }
    out.rigid_bodies.push_back(body);
  }

  for (const auto & sk : frame.skeletons) {
    Skeleton skel;
    skel.id = sk.id;
    const auto desc = model.skeletons.find(sk.id);
    skel.name = desc != model.skeletons.end() ? desc->second.name : std::to_string(sk.id);
    skel.tracking_valid = !sk.bones.empty();

    std::map<int32_t, const natnet::RigidBodyDescription *> bone_desc;
    if (desc != model.skeletons.end()) {
      for (const auto & b : desc->second.bones) {
        bone_desc[b.id] = &b;
      }
    }
    // Bones may come in any order; local poses compose with their parents.
    std::map<int32_t, Pose> world;
    std::map<int32_t, const natnet::RigidBodyData *> data;
    for (const auto & b : sk.bones) {
      data[b.id & 0xffff] = &b;
    }
    std::function<Pose(int32_t, int)> global_pose = [&](int32_t id, int depth) -> Pose {
        const auto known = world.find(id);
        if (known != world.end()) {
          return known->second;
        }
        const auto * b = data.at(id);
        Pose p = make_pose(b->position, b->orientation, options.up_axis);
        const auto d = bone_desc.find(id);
        if (options.skeleton_local && d != bone_desc.end() && depth < 64 &&
          data.count(d->second->parent_id) && d->second->parent_id != id)
        {
          p = compose(global_pose(d->second->parent_id, depth + 1), p);
        }
        world[id] = p;
        return p;
      };

    for (const auto & b : sk.bones) {
      const int32_t id = b.id & 0xffff;
      Segment seg;
      seg.id = id;
      const auto d = bone_desc.find(id);
      seg.name = d != bone_desc.end() ? d->second->name : std::to_string(id);
      seg.parent_id = d != bone_desc.end() && d->second->parent_id > 0 &&
        d->second->parent_id != id ? d->second->parent_id : Segment::NO_PARENT;
      seg.tracking_valid = b.tracking_valid();
      seg.pose = to_msg(global_pose(id, 0));
      skel.tracking_valid = skel.tracking_valid && seg.tracking_valid;
      skel.segments.push_back(seg);
    }
    out.skeletons.push_back(skel);
  }
  return out;
}

}  // namespace mocap4r2_optitrack_driver
