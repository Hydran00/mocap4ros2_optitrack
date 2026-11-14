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

// natnet_simulator: a stand-in for Motive that streams moving rigid bodies,
// to check the driver, the network and the rest of the pipeline without a lab.

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "mocap4r2_optitrack_driver/fake_natnet_server.hpp"

using mocap4r2_optitrack_driver::FakeNatNetServer;
namespace natnet = mocap4r2_optitrack_driver::natnet;

namespace
{

volatile std::sig_atomic_t g_stop = 0;

void usage()
{
  std::cout <<
    "usage: natnet_simulator [options]\n"
    "  --address IP        command socket address (default 127.0.0.1)\n"
    "  --command-port N    (default 1510)\n"
    "  --data-port N       (default 1511)\n"
    "  --multicast         stream to 239.255.42.99 instead of unicast\n"
    "  --version M.m       NatNet bitstream version (default 4.4)\n"
    "  --rate HZ           frame rate (default 120)\n"
    "  --bodies N          moving rigid bodies (default 2)\n"
    "  --up-axis y|z       axis convention of the streamed data (default y)\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  FakeNatNetServer::Config config;
  double rate = 120.0;
  int bodies = 2;
  bool y_up = true;
  for (int i = 1; i < argc; i++) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {
        if (i + 1 >= argc) {
          usage();
          std::exit(1);
        }
        return argv[++i];
      };
    if (a == "--address") {
      config.address = next();
    } else if (a == "--command-port") {
      config.command_port = static_cast<uint16_t>(std::stoi(next()));
    } else if (a == "--data-port") {
      config.data_port = static_cast<uint16_t>(std::stoi(next()));
    } else if (a == "--multicast") {
      config.multicast = true;
    } else if (a == "--version") {
      const auto v = next();
      const auto dot = v.find('.');
      config.version = {std::stoi(v.substr(0, dot)),
        dot == std::string::npos ? 0 : std::stoi(v.substr(dot + 1))};
    } else if (a == "--rate") {
      rate = std::stod(next());
    } else if (a == "--bodies") {
      bodies = std::stoi(next());
    } else if (a == "--up-axis") {
      y_up = next() != "z";
    } else {
      usage();
      return a == "--help" || a == "-h" ? 0 : 1;
    }
  }
  if (rate <= 0.0 || bodies < 0 || bodies > 100 || config.version.major < 3) {
    usage();
    return 1;
  }

  natnet::DataDescriptions desc;
  for (int b = 0; b < bodies; b++) {
    natnet::RigidBodyDescription rb;
    rb.name = "body" + std::to_string(b + 1);
    rb.id = b + 1;
    rb.marker_names = {"m1", "m2", "m3"};
    desc.rigid_bodies.push_back(rb);
  }

  std::signal(SIGINT, [](int) {g_stop = 1;});
  std::signal(SIGTERM, [](int) {g_stop = 1;});
  FakeNatNetServer server(config, desc);
  std::printf(
    "NatNet %d.%d simulator on %s:%u (%s), %d bodies at %.0f Hz\n", config.version.major,
    config.version.minor, config.address.c_str(), config.command_port,
    config.multicast ? "multicast" : "unicast", bodies, rate);

  const auto period = std::chrono::duration<double>(1.0 / rate);
  auto next = std::chrono::steady_clock::now();
  const auto start = next;
  int32_t frame_number = 0;
  while (!g_stop) {
    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    std::this_thread::sleep_until(next);
    const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() -
      start).count();
    natnet::FrameData f;
    f.frame_number = ++frame_number;
    for (int b = 0; b < bodies; b++) {
      const double w = 2.0 * M_PI / (10.0 + 5.0 * b);
      const double r = 1.0 + 0.5 * b;
      const double h = 0.2 + 0.5 * b;
      // Ground-plane motion; (x, y, z) world Z-up streamed as Y-up when asked.
      const double x = r * std::cos(w * t), y = r * std::sin(w * t);
      const double yaw = w * t + M_PI_2;
      natnet::RigidBodyData rb;
      rb.id = b + 1;
      rb.params = natnet::RB_TRACKING_VALID;
      rb.mean_error = 0.0004f;
      if (y_up) {
        rb.position = {static_cast<float>(x), static_cast<float>(h), static_cast<float>(-y)};
        rb.orientation = {0.0f, static_cast<float>(std::sin(yaw / 2)), 0.0f,
          static_cast<float>(std::cos(yaw / 2))};
      } else {
        rb.position = {static_cast<float>(x), static_cast<float>(y), static_cast<float>(h)};
        rb.orientation = {0.0f, 0.0f, static_cast<float>(std::sin(yaw / 2)),
          static_cast<float>(std::cos(yaw / 2))};
      }
      f.rigid_bodies.push_back(rb);
      for (int m = 0; m < 3; m++) {
        natnet::LabeledMarker lm;
        lm.id = ((b + 1) << 16) | (m + 1);
        lm.position = {rb.position[0] + 0.05f * m, rb.position[1], rb.position[2]};
        lm.size = 0.014f;
        lm.residual = 0.0003f;
        f.labeled_markers.push_back(lm);
      }
    }
    const auto ticks = static_cast<uint64_t>(t * config.clock_frequency);
    f.camera_mid_exposure = ticks;
    f.camera_data_received = ticks + config.clock_frequency / 500;
    f.transmit = ticks + config.clock_frequency / 250;  // 4 ms of Motive latency
    f.timestamp = t;
    server.send_frame(f);
  }
  return 0;
}
