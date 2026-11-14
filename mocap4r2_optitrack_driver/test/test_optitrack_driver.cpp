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

#include <unistd.h>

#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "mocap4r2_interfaces/msg/markers.hpp"
#include "mocap4r2_interfaces/msg/rigid_bodies.hpp"
#include "mocap4r2_optitrack_driver/fake_natnet_server.hpp"
#include "mocap4r2_optitrack_driver/natnet_backend.hpp"
#include "mocap4r2_optitrack_driver/optitrack_driver.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;
using mocap4r2_interfaces::msg::Markers;
using mocap4r2_interfaces::msg::RigidBodies;
using mocap4r2_optitrack_driver::FakeNatNetServer;
using mocap4r2_optitrack_driver::OptitrackDriver;
namespace natnet = mocap4r2_optitrack_driver::natnet;

namespace
{

uint16_t random_port()
{
  static std::mt19937 rng(static_cast<unsigned>(getpid()) * 7919u);
  // Below the kernel ephemeral range (32768+): never collides with client sockets.
  return static_cast<uint16_t>(std::uniform_int_distribution<int>(20000, 32000)(rng));
}

natnet::DataDescriptions robot_descriptions(const std::string & name = "robot")
{
  natnet::DataDescriptions d;
  natnet::RigidBodyDescription rb;
  rb.name = name;
  rb.id = 1;
  rb.marker_names = {"a", "b"};
  d.rigid_bodies.push_back(rb);
  return d;
}

natnet::FrameData frame(int32_t n, bool models_changed = false)
{
  natnet::FrameData f;
  f.frame_number = n;
  natnet::RigidBodyData rb;
  rb.id = 1;
  rb.position = {1.0f, 0.5f, -2.0f};  // Y-up: z_up = (1, 2, 0.5)
  rb.params = natnet::RB_TRACKING_VALID;
  f.rigid_bodies = {rb};
  natnet::LabeledMarker m;
  m.id = (1 << 16) | 2;
  f.labeled_markers = {m};
  f.camera_mid_exposure = 1000000;
  f.transmit = 1000000 + 50000;  // 5 ms at 10 MHz
  if (models_changed) {
    f.params = natnet::FRAME_MODELS_CHANGED;
  }
  return f;
}

}  // namespace

class OptitrackDriverTest : public ::testing::TestWithParam<std::string>
{
protected:
  void SetUp() override
  {
    port_ = random_port();
    client_ = rclcpp::Node::make_shared("optitrack_test_client");
    sub_ = client_->create_subscription<RigidBodies>(
      "/rigid_bodies", rclcpp::QoS(100).reliable(),
      [this](RigidBodies::ConstSharedPtr m) {bodies_.push_back(*m);});
    markers_sub_ = client_->create_subscription<Markers>(
      "/markers", rclcpp::QoS(100).reliable(),
      [this](Markers::ConstSharedPtr m) {markers_.push_back(*m);});
    executor_.add_node(client_);
  }

  void TearDown() override
  {
    if (driver_) {
      executor_.remove_node(driver_->get_node_base_interface());
      driver_.reset();
    }
    server_.reset();
  }

  void start_server(bool multicast = false)
  {
    FakeNatNetServer::Config c;
    c.command_port = port_;
    c.data_port = static_cast<uint16_t>(port_ + 1);
    c.multicast = multicast;
    server_ = std::make_unique<FakeNatNetServer>(c, robot_descriptions());
  }

  std::shared_ptr<OptitrackDriver> make_driver(std::vector<rclcpp::Parameter> extra = {})
  {
    std::vector<rclcpp::Parameter> p = {
      {"server_address", "127.0.0.1"}, {"server_command_port", static_cast<int>(port_)},
      {"server_data_port", static_cast<int>(port_ + 1)}, {"local_address", "127.0.0.1"},
      {"timeout", 0.5}, {"backend", backend()}};
    if (backend() == "sdk") {
      p.push_back({"connection_type", "unicast"});
    }
    p.insert(p.end(), extra.begin(), extra.end());
    driver_ = std::make_shared<OptitrackDriver>(rclcpp::NodeOptions().parameter_overrides(p));
    executor_.add_node(driver_->get_node_base_interface());
    return driver_;
  }

  bool spin_until(const std::function<bool()> & cond, std::chrono::seconds timeout = 10s)
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      executor_.spin_some(10ms);
      if (cond()) {
        return true;
      }
    }
    return cond();
  }

  // Stream frames until cond holds (UDP: some may be lost before the client is ready).
  bool stream_until(const std::function<bool()> & cond, int32_t & n, bool changed = false)
  {
    const auto end = std::chrono::steady_clock::now() + 10s;
    while (std::chrono::steady_clock::now() < end) {
      server_->send_frame(frame(++n, changed));
      executor_.spin_some(10ms);
      if (cond()) {
        return true;
      }
    }
    return false;
  }

  std::string backend() const {return GetParam();}

  uint16_t port_{0};
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr client_;
  std::shared_ptr<OptitrackDriver> driver_;
  std::unique_ptr<FakeNatNetServer> server_;
  rclcpp::Subscription<RigidBodies>::SharedPtr sub_;
  rclcpp::Subscription<Markers>::SharedPtr markers_sub_;
  std::vector<RigidBodies> bodies_;
  std::vector<Markers> markers_;
};

TEST_P(OptitrackDriverTest, configure_fails_without_server)
{
  auto d = make_driver();
  const auto t0 = std::chrono::steady_clock::now();
  d->trigger_transition(Transition::TRANSITION_CONFIGURE);
  EXPECT_EQ(d->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 5s);  // bounded by the timeout
}

TEST_P(OptitrackDriverTest, unicast_stream_end_to_end)
{
  start_server();
  auto d = make_driver({{"frame_id", "lab"}});
  d->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(d->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_GE(server_->connect_requests(), 1);
  EXPECT_GE(server_->description_requests(), 1);
  if (backend() == "open") {
    EXPECT_EQ(server_->requested_version().major, 4);
  }

  d->trigger_transition(Transition::TRANSITION_ACTIVATE);
  ASSERT_TRUE(spin_until([&] {return server_->clients() > 0;}));  // client registered
  int32_t n = 100;
  ASSERT_TRUE(stream_until([&] {return bodies_.size() >= 5 && !markers_.empty();}, n));

  const auto & b = bodies_.back();
  EXPECT_EQ(b.header.frame_id, "lab");
  ASSERT_EQ(b.rigid_bodies.size(), 1u);
  const auto & rb = b.rigid_bodies[0];
  EXPECT_EQ(rb.name, "robot");
  EXPECT_TRUE(rb.tracking_valid);
  EXPECT_NEAR(rb.pose.position.x, 1.0, 1e-6);
  EXPECT_NEAR(rb.pose.position.y, 2.0, 1e-6);
  EXPECT_NEAR(rb.pose.position.z, 0.5, 1e-6);
  ASSERT_EQ(rb.markers.size(), 1u);
  EXPECT_EQ(rb.markers[0].name, "robot:b");
  EXPECT_GT(b.frame_number, 100u);
  // Stamp = reception - 5 ms of Motive latency.
  const double age = (client_->now() - rclcpp::Time(b.header.stamp)).seconds();
  EXPECT_GE(age, 0.004);
  EXPECT_LT(age, 1.0);
  // Markers and bodies of a frame share stamp and number.
  for (const auto & m : markers_) {
    if (m.frame_number == b.frame_number) {
      EXPECT_EQ(m.header.stamp, b.header.stamp);
    }
  }

  d->trigger_transition(Transition::TRANSITION_DEACTIVATE);
  spin_until([] {return false;}, 1s);  // drain messages published before deactivation
  const auto count = bodies_.size();
  for (int i = 0; i < 20; i++) {
    server_->send_frame(frame(++n));
    executor_.spin_some(10ms);
  }
  EXPECT_EQ(bodies_.size(), count);
}

TEST_P(OptitrackDriverTest, malformed_frames_do_not_stop_the_stream)
{
  start_server();
  auto d = make_driver();
  d->trigger_transition(Transition::TRANSITION_CONFIGURE);
  d->trigger_transition(Transition::TRANSITION_ACTIVATE);
  ASSERT_TRUE(spin_until([&] {return server_->clients() > 0;}));
  for (int i = 0; i < 50; i++) {
    auto bad = natnet::encode_frame(frame(i), {4, 4});
    bad.resize(bad.size() / 2);
    server_->send_raw(bad);
    server_->send_raw({7, 0});
    server_->send_raw({7, 0, 0xff, 0xff, 1, 2, 3});
  }
  int32_t n = 1000;
  ASSERT_TRUE(stream_until([&] {return !bodies_.empty();}, n));
  EXPECT_EQ(d->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
}

TEST_P(OptitrackDriverTest, model_changes_reload_names)
{
  start_server();
  auto d = make_driver();
  d->trigger_transition(Transition::TRANSITION_CONFIGURE);
  d->trigger_transition(Transition::TRANSITION_ACTIVATE);
  ASSERT_TRUE(spin_until([&] {return server_->clients() > 0;}));

  server_->set_descriptions(robot_descriptions("renamed"));
  int32_t n = 0;
  ASSERT_TRUE(
    stream_until(
      [&] {
        return !bodies_.empty() && bodies_.back().rigid_bodies[0].name == "renamed";
      }, n, true));
  EXPECT_GE(server_->description_requests(), 2);
}

TEST_P(OptitrackDriverTest, reconfigure_cycles_and_destroy_while_active)
{
  start_server();
  auto d = make_driver();
  for (int i = 0; i < 3; i++) {
    d->trigger_transition(Transition::TRANSITION_CONFIGURE);
    ASSERT_EQ(d->get_current_state().id(), State::PRIMARY_STATE_INACTIVE) << i;
    d->trigger_transition(Transition::TRANSITION_ACTIVATE);
    bodies_.clear();
    int32_t n = i * 1000;
    ASSERT_TRUE(stream_until([&] {return !bodies_.empty();}, n)) << i;
    d->trigger_transition(Transition::TRANSITION_DEACTIVATE);
    d->trigger_transition(Transition::TRANSITION_CLEANUP);
  }
  EXPECT_GE(server_->connect_requests(), 3);
  d->trigger_transition(Transition::TRANSITION_CONFIGURE);
  d->trigger_transition(Transition::TRANSITION_ACTIVATE);
  // Destroy while frames are arriving: the receive thread must stop first.
  std::thread sender([&] {
      for (int i = 0; i < 200; i++) {
        server_->send_frame(frame(5000 + i));
        std::this_thread::sleep_for(1ms);
      }
    });
  executor_.remove_node(d->get_node_base_interface());
  d.reset();
  driver_.reset();
  sender.join();
  SUCCEED();
}

TEST_P(OptitrackDriverTest, invalid_parameters_fail_configure)
{
  start_server();
  const std::vector<std::vector<rclcpp::Parameter>> bad = {
    {{"connection_type", "broadcast"}},
    {{"up_axis", "x"}},
    {{"skeleton_coordinates", "relative"}},
    {{"server_address", "not.an.ip"}},
    {{"local_address", "999.0.0.1"}},
    {{"server_data_port", 70000}},
    {{"timeout", 0.0}},
  };
  for (size_t i = 0; i < bad.size(); i++) {
    auto d = make_driver(bad[i]);
    d->trigger_transition(Transition::TRANSITION_CONFIGURE);
    EXPECT_EQ(d->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED) << i;
    executor_.remove_node(d->get_node_base_interface());
    driver_.reset();
  }
  EXPECT_EQ(server_->connect_requests(), 0);  // rejected before talking to Motive
}

TEST_P(OptitrackDriverTest, multicast_stream)
{
  start_server(true);
  auto d = make_driver({{"connection_type", "multicast"}});
  d->trigger_transition(Transition::TRANSITION_CONFIGURE);
  if (d->get_current_state().id() != State::PRIMARY_STATE_INACTIVE) {
    GTEST_SKIP() << "multicast on loopback not available";
  }
  d->trigger_transition(Transition::TRANSITION_ACTIVATE);
  if (d->get_current_state().id() != State::PRIMARY_STATE_ACTIVE) {
    GTEST_SKIP() << "cannot join multicast group on loopback";
  }
  int32_t n = 0;
  if (!stream_until([&] {return !bodies_.empty();}, n)) {
    GTEST_SKIP() << "multicast loopback delivers no data here";
  }
  EXPECT_EQ(bodies_.back().rigid_bodies[0].name, "robot");
}

std::vector<std::string> backends()
{
  std::vector<std::string> b = {"open"};
  if (mocap4r2_optitrack_driver::sdk_backend_available()) {
    b.push_back("sdk");
  }
  return b;
}

INSTANTIATE_TEST_SUITE_P(
  Backends, OptitrackDriverTest, ::testing::ValuesIn(backends()),
  [](const ::testing::TestParamInfo<std::string> & i) {return i.param;});

TEST(OptitrackBackends, sdk_requested_without_sdk_fails_configure)
{
  if (mocap4r2_optitrack_driver::sdk_backend_available()) {
    GTEST_SKIP() << "built with the NatNet SDK";
  }
  auto d = std::make_shared<OptitrackDriver>(
    rclcpp::NodeOptions().parameter_overrides({{"backend", "sdk"}}));
  d->trigger_transition(Transition::TRANSITION_CONFIGURE);
  EXPECT_EQ(d->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_THROW(
    mocap4r2_optitrack_driver::make_backend(
      mocap4r2_optitrack_driver::BackendKind::SDK, {}), std::invalid_argument);
  auto open = mocap4r2_optitrack_driver::make_backend(
    mocap4r2_optitrack_driver::BackendKind::AUTO, {});
  EXPECT_EQ(open->name(), "open");
}

// Both backends must turn the same stream into the same messages.
TEST(OptitrackBackends, sdk_and_open_produce_the_same_messages)
{
  if (!mocap4r2_optitrack_driver::sdk_backend_available()) {
    GTEST_SKIP() << "built without the NatNet SDK";
  }
  const uint16_t port = random_port();
  FakeNatNetServer::Config c;
  c.command_port = port;
  c.data_port = static_cast<uint16_t>(port + 1);
  natnet::DataDescriptions desc = robot_descriptions();
  natnet::SkeletonDescription sk;
  sk.name = "subject";
  sk.id = 7;
  for (int i = 1; i <= 2; i++) {
    natnet::RigidBodyDescription b;
    b.name = "bone" + std::to_string(i);
    b.id = i;
    b.parent_id = i - 1;
    sk.bones.push_back(b);
  }
  desc.skeletons.push_back(sk);
  FakeNatNetServer server(c, desc);

  auto collect = [&](const std::string & backend) {
      rclcpp::executors::SingleThreadedExecutor exec;
      auto node = rclcpp::Node::make_shared("equiv_" + backend);
      std::vector<RigidBodies> out;
      auto sub = node->create_subscription<RigidBodies>(
        "/" + backend + "/rigid_bodies", rclcpp::QoS(100).reliable(),
        [&](RigidBodies::ConstSharedPtr m) {out.push_back(*m);});
      auto d = std::make_shared<OptitrackDriver>(
        rclcpp::NodeOptions().parameter_overrides(
          {{"backend", backend}, {"server_address", "127.0.0.1"},
            {"server_command_port", static_cast<int>(port)},
            {"server_data_port", static_cast<int>(port + 1)}, {"local_address", "127.0.0.1"},
            {"connection_type", "unicast"}, {"compensate_latency", false}})
        .arguments({"--ros-args", "-r", "__ns:=/" + backend}));
      exec.add_node(node);
      exec.add_node(d->get_node_base_interface());
      d->trigger_transition(Transition::TRANSITION_CONFIGURE);
      d->trigger_transition(Transition::TRANSITION_ACTIVATE);
      EXPECT_EQ(d->get_current_state().id(), State::PRIMARY_STATE_ACTIVE) << backend;
      int32_t n = 0;
      const auto end = std::chrono::steady_clock::now() + 10s;
      while (out.size() < 5 && std::chrono::steady_clock::now() < end) {
        natnet::FrameData f = frame(++n);
        natnet::SkeletonData s;
        s.id = 7;
        natnet::RigidBodyData b1 = f.rigid_bodies[0], b2 = f.rigid_bodies[0];
        b1.id = (7 << 16) | 1;
        b2.id = (7 << 16) | 2;
        s.bones = {b1, b2};
        f.skeletons = {s};
        server.send_frame(f);
        exec.spin_some(20ms);
      }
      exec.remove_node(d->get_node_base_interface());
      d->shutdown();
      return out;
    };

  const auto open = collect("open");
  const auto sdk = collect("sdk");
  ASSERT_FALSE(open.empty());
  ASSERT_FALSE(sdk.empty());
  // Compare the content (frame numbers and stamps differ between runs).
  auto a = open.back();
  auto b = sdk.back();
  a.header.stamp = b.header.stamp;
  a.frame_number = b.frame_number;
  EXPECT_EQ(a, b);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
