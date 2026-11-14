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

#include <cmath>
#include <string>

#include "mocap4r2_optitrack_driver/frame_converter.hpp"

namespace natnet = mocap4r2_optitrack_driver::natnet;
using mocap4r2_interfaces::msg::Marker;
using mocap4r2_interfaces::msg::Segment;
using mocap4r2_optitrack_driver::ConverterOptions;
using mocap4r2_optitrack_driver::convert_frame;
using mocap4r2_optitrack_driver::frame_latency;
using mocap4r2_optitrack_driver::Model;
using mocap4r2_optitrack_driver::UpAxis;

namespace
{

Model robot_model()
{
  natnet::DataDescriptions d;
  natnet::RigidBodyDescription rb;
  rb.name = "robot";
  rb.id = 3;
  rb.marker_names = {"front", "left", "right"};
  d.rigid_bodies.push_back(rb);
  natnet::SkeletonDescription sk;
  sk.name = "subject";
  sk.id = 7;
  for (int i = 1; i <= 3; i++) {
    natnet::RigidBodyDescription b;
    b.name = "bone" + std::to_string(i);
    b.id = i;
    b.parent_id = i - 1;  // 0: root has no parent
    sk.bones.push_back(b);
  }
  d.skeletons.push_back(sk);
  return Model::from(d);
}

natnet::LabeledMarker lm(int asset, int member, int16_t params = 0)
{
  natnet::LabeledMarker m;
  m.id = (asset << 16) | member;
  m.params = params;
  m.position = {1.0f, 2.0f, 3.0f};
  m.residual = 0.001f;
  return m;
}

natnet::RigidBodyData rb(int id, std::array<float, 3> p, std::array<float, 4> q, bool valid = true)
{
  natnet::RigidBodyData r;
  r.id = id;
  r.position = p;
  r.orientation = q;
  r.params = valid ? natnet::RB_TRACKING_VALID : 0;
  r.mean_error = 0.0005f;
  return r;
}

ConverterOptions z_up()
{
  ConverterOptions o;
  o.up_axis = UpAxis::Z;
  return o;
}

}  // namespace

TEST(FrameConverter, names_ids_and_markers_of_bodies)
{
  natnet::FrameData f;
  f.frame_number = 42;
  f.rigid_bodies = {rb(3, {1, 2, 3}, {0, 0, 0, 1}), rb(99, {0, 0, 0}, {0, 0, 0, 1}, false)};
  f.labeled_markers = {lm(3, 2), lm(3, 9), lm(3, 1, natnet::MARKER_OCCLUDED), lm(0, 5),
    lm(3, 3, natnet::MARKER_UNLABELED), lm(55, 1)};
  const auto out = convert_frame(f, robot_model(), z_up());
  EXPECT_EQ(out.frame_number, 42u);
  ASSERT_EQ(out.rigid_bodies.size(), 2u);
  EXPECT_EQ(out.rigid_bodies[0].name, "robot");
  EXPECT_EQ(out.rigid_bodies[0].id, 3);
  EXPECT_TRUE(out.rigid_bodies[0].tracking_valid);
  EXPECT_FLOAT_EQ(out.rigid_bodies[0].mean_error, 0.0005f);
  EXPECT_EQ(out.rigid_bodies[1].name, "99");  // unknown asset: id as name
  EXPECT_FALSE(out.rigid_bodies[1].tracking_valid);

  ASSERT_EQ(out.markers.size(), 6u);
  EXPECT_EQ(out.markers[0].name, "robot:left");
  EXPECT_EQ(out.markers[0].id, (3 << 16) | 2);
  EXPECT_EQ(out.markers[1].name, "robot:9");      // member beyond the description
  EXPECT_EQ(out.markers[2].name, "robot:front");
  EXPECT_TRUE(out.markers[2].occluded);
  EXPECT_EQ(out.markers[3].id, Marker::UNKNOWN_ID);  // asset 0: point cloud
  EXPECT_TRUE(out.markers[3].name.empty());
  EXPECT_EQ(out.markers[4].id, Marker::UNKNOWN_ID);  // unlabeled flag
  EXPECT_EQ(out.markers[5].name, "");                // unknown asset, but labeled
  EXPECT_EQ(out.markers[5].id, (55 << 16) | 1);
  EXPECT_FLOAT_EQ(out.markers[0].residual, 0.001f);

  // The body carries its own labeled markers only.
  ASSERT_EQ(out.rigid_bodies[0].markers.size(), 3u);
  EXPECT_EQ(out.rigid_bodies[0].markers[0].name, "robot:left");
  EXPECT_TRUE(out.rigid_bodies[1].markers.empty());
}

TEST(FrameConverter, z_up_passthrough)
{
  natnet::FrameData f;
  const float s = std::sin(0.25f), c = std::cos(0.25f);
  f.rigid_bodies = {rb(3, {1, 2, 3}, {0, 0, s, c})};
  const auto out = convert_frame(f, robot_model(), z_up());
  const auto & p = out.rigid_bodies[0].pose;
  EXPECT_DOUBLE_EQ(p.position.x, 1.0);
  EXPECT_DOUBLE_EQ(p.position.y, 2.0);
  EXPECT_DOUBLE_EQ(p.position.z, 3.0);
  EXPECT_NEAR(p.orientation.z, s, 1e-7);
  EXPECT_NEAR(p.orientation.w, c, 1e-7);
}

TEST(FrameConverter, y_up_is_rotated_to_z_up)
{
  // A body 1.5 m above the floor (Motive Y) at Motive X=2, Z=-1, yawed 0.5 rad
  // about the vertical, must be at (2, 1, 1.5) yawed 0.5 rad about Z.
  natnet::FrameData f;
  const float s = std::sin(0.25f), c = std::cos(0.25f);
  f.rigid_bodies = {rb(3, {2, 1.5f, -1}, {0, s, 0, c})};
  f.labeled_markers = {lm(3, 1)};
  f.labeled_markers[0].position = {2, 1.5f, -1};
  const auto out = convert_frame(f, robot_model(), ConverterOptions{});
  const auto & p = out.rigid_bodies[0].pose;
  EXPECT_NEAR(p.position.x, 2.0, 1e-6);
  EXPECT_NEAR(p.position.y, 1.0, 1e-6);
  EXPECT_NEAR(p.position.z, 1.5, 1e-6);
  EXPECT_NEAR(p.orientation.x, 0.0, 1e-7);
  EXPECT_NEAR(p.orientation.y, 0.0, 1e-7);
  EXPECT_NEAR(p.orientation.z, s, 1e-7);
  EXPECT_NEAR(p.orientation.w, c, 1e-7);
  EXPECT_NEAR(out.markers[0].position.y, 1.0, 1e-6);
  EXPECT_NEAR(out.markers[0].position.z, 1.5, 1e-6);
}

TEST(FrameConverter, y_up_tilt_keeps_its_meaning)
{
  // Pitch-like tilt about Motive X stays a rotation about X (shared axis).
  natnet::FrameData f;
  const float s = std::sin(0.2f), c = std::cos(0.2f);
  f.rigid_bodies = {rb(3, {0, 0, 0}, {s, 0, 0, c})};
  const auto out = convert_frame(f, robot_model(), ConverterOptions{});
  EXPECT_NEAR(out.rigid_bodies[0].pose.orientation.x, s, 1e-7);
  EXPECT_NEAR(out.rigid_bodies[0].pose.orientation.w, c, 1e-7);
}

TEST(FrameConverter, quaternions_are_normalized)
{
  natnet::FrameData f;
  f.rigid_bodies = {rb(3, {0, 0, 0}, {0, 0, 2, 2}), rb(4, {0, 0, 0}, {0, 0, 0, 0})};
  const auto out = convert_frame(f, robot_model(), z_up());
  EXPECT_NEAR(out.rigid_bodies[0].pose.orientation.z, std::sqrt(0.5), 1e-7);
  EXPECT_DOUBLE_EQ(out.rigid_bodies[1].pose.orientation.w, 1.0);  // zero -> identity
}

TEST(FrameConverter, skeleton_global_and_local)
{
  natnet::FrameData f;
  natnet::SkeletonData sk;
  sk.id = 7;
  // Each bone 0.5 m above its parent in local coordinates; root at z = 1.
  // Root yawed 90 degrees: children offsets along local x end up along world y.
  const float s = std::sin(M_PI_4), c = std::cos(M_PI_4);
  sk.bones = {rb((7 << 16) | 1, {0, 0, 1}, {0, 0, s, c}), rb((7 << 16) | 2, {0.5f, 0, 0},
      {0, 0, 0, 1}), rb((7 << 16) | 3, {0.5f, 0, 0}, {0, 0, 0, 1})};
  f.skeletons = {sk};

  auto global = convert_frame(f, robot_model(), z_up());
  ASSERT_EQ(global.skeletons.size(), 1u);
  const auto & g = global.skeletons[0];
  EXPECT_EQ(g.name, "subject");
  ASSERT_EQ(g.segments.size(), 3u);
  EXPECT_EQ(g.segments[0].name, "bone1");
  EXPECT_EQ(g.segments[0].parent_id, Segment::NO_PARENT);
  EXPECT_EQ(g.segments[2].parent_id, 2);
  EXPECT_DOUBLE_EQ(g.segments[2].pose.position.x, 0.5);  // taken as is

  auto o = z_up();
  o.skeleton_local = true;
  auto local = convert_frame(f, robot_model(), o);
  const auto & l = local.skeletons[0];
  EXPECT_NEAR(l.segments[1].pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(l.segments[1].pose.position.y, 0.5, 1e-6);
  EXPECT_NEAR(l.segments[1].pose.position.z, 1.0, 1e-6);
  EXPECT_NEAR(l.segments[2].pose.position.y, 1.0, 1e-6);
  EXPECT_NEAR(l.segments[2].pose.orientation.z, s, 1e-6);  // inherits the yaw
  EXPECT_TRUE(l.tracking_valid);
}

TEST(FrameConverter, skeleton_out_of_order_unknown_and_cyclic)
{
  natnet::FrameData f;
  natnet::SkeletonData sk;
  sk.id = 7;
  // Child before parent, plus a bone not in the description.
  sk.bones = {rb((7 << 16) | 2, {0, 0, 0.5f}, {0, 0, 0, 1}), rb((7 << 16) | 1, {0, 0, 1},
      {0, 0, 0, 1}), rb((7 << 16) | 9, {0, 0, 0}, {0, 0, 0, 1}, false)};
  f.skeletons = {sk};
  auto o = z_up();
  o.skeleton_local = true;
  auto out = convert_frame(f, robot_model(), o);
  const auto & s = out.skeletons[0];
  EXPECT_NEAR(s.segments[0].pose.position.z, 1.5, 1e-6);
  EXPECT_EQ(s.segments[2].name, "9");
  EXPECT_EQ(s.segments[2].parent_id, Segment::NO_PARENT);
  EXPECT_FALSE(s.tracking_valid);  // one bone lost

  // A description where bones are parents of each other must not loop forever.
  natnet::DataDescriptions d;
  natnet::SkeletonDescription cyc;
  cyc.name = "cyclic";
  cyc.id = 8;
  natnet::RigidBodyDescription a, b;
  a.id = 1;
  a.parent_id = 2;
  b.id = 2;
  b.parent_id = 1;
  cyc.bones = {a, b};
  d.skeletons = {cyc};
  natnet::FrameData cf;
  natnet::SkeletonData csk;
  csk.id = 8;
  csk.bones = {rb((8 << 16) | 1, {0, 0, 1}, {0, 0, 0, 1}), rb((8 << 16) | 2, {0, 0, 1},
      {0, 0, 0, 1})};
  cf.skeletons = {csk};
  auto cout = convert_frame(cf, Model::from(d), o);
  EXPECT_EQ(cout.skeletons[0].segments.size(), 2u);
}

TEST(FrameConverter, unknown_skeleton)
{
  natnet::FrameData f;
  natnet::SkeletonData sk;
  sk.id = 77;
  f.skeletons = {sk};
  const auto out = convert_frame(f, Model{}, z_up());
  EXPECT_EQ(out.skeletons[0].name, "77");
  EXPECT_FALSE(out.skeletons[0].tracking_valid);  // no bones
}

TEST(FrameConverter, latency)
{
  natnet::FrameData f;
  f.camera_mid_exposure = 1000000;
  f.transmit = 1040000;
  EXPECT_DOUBLE_EQ(frame_latency(f, 10000000), 0.004);
  EXPECT_DOUBLE_EQ(frame_latency(f, 0), 0.0);         // unknown clock
  f.transmit = 10;
  EXPECT_DOUBLE_EQ(frame_latency(f, 10000000), 0.0);  // clock wrap / bad data
  f.camera_mid_exposure = 0;
  EXPECT_DOUBLE_EQ(frame_latency(f, 10000000), 0.0);
}

TEST(FrameConverter, frame_number_is_unsigned)
{
  natnet::FrameData f;
  f.frame_number = -1;  // wrapped int32 in Motive
  EXPECT_EQ(convert_frame(f, Model{}, z_up()).frame_number, 0xffffffffu);
}
