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
#include <cctype>
#include <memory>
#include <string>
#include <utility>

#include "mocap4r2_optitrack_driver/optitrack_driver.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace mocap4r2_optitrack_driver
{

namespace
{

std::string lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {return std::tolower(c);});
  return s;
}

std::string version_string(const std::array<uint8_t, 4> & v)
{
  return std::to_string(v[0]) + "." + std::to_string(v[1]) + "." + std::to_string(v[2]) + "." +
         std::to_string(v[3]);
}

}  // namespace

OptitrackDriver::OptitrackDriver(const rclcpp::NodeOptions & options)
: MocapSystemNode("optitrack_driver", "optitrack", options)
{
  declare_parameter<std::string>("backend", "auto");
  declare_parameter<std::string>("server_address", "127.0.0.1");
  declare_parameter<std::string>("local_address", "0.0.0.0");
  declare_parameter<std::string>("connection_type", "auto");
  declare_parameter<std::string>("multicast_address", "");
  declare_parameter<int>("server_command_port", 1510);
  declare_parameter<int>("server_data_port", 0);
  declare_parameter<double>("timeout", 2.0);
  declare_parameter<std::string>("up_axis", "y");
  declare_parameter<std::string>("skeleton_coordinates", "global");
  declare_parameter<bool>("compensate_latency", true);
  mocap4r2_control::declare_frame_publisher_parameters(*this);
}

OptitrackDriver::~OptitrackDriver()
{
  // Stop the receive and refresh threads before the node goes away.
  release_client();
}

void
OptitrackDriver::release_client()
{
  if (client_) {
    client_->stop();
  }
  if (refresh_thread_.joinable()) {
    refresh_thread_.join();
  }
  if (client_) {
    client_->disconnect();
  }
  client_.reset();
}

OptitrackDriver::CallbackReturn
OptitrackDriver::on_system_configure()
{
  NatNetConfig config;
  config.server_address = get_parameter("server_address").as_string();
  config.local_address = get_parameter("local_address").as_string();
  config.multicast_address = get_parameter("multicast_address").as_string();
  const auto cmd_port = get_parameter("server_command_port").as_int();
  const auto data_port = get_parameter("server_data_port").as_int();
  if (cmd_port < 1 || cmd_port > 65535 || data_port < 0 || data_port > 65535) {
    RCLCPP_ERROR(get_logger(), "ports must be in [1, 65535] (server_data_port 0: from server)");
    return CallbackReturn::FAILURE;
  }
  config.command_port = static_cast<uint16_t>(cmd_port);
  config.data_port = static_cast<uint16_t>(data_port);
  const double timeout = get_parameter("timeout").as_double();
  if (timeout <= 0.0) {
    RCLCPP_ERROR(get_logger(), "timeout must be > 0");
    return CallbackReturn::FAILURE;
  }
  config.timeout = std::chrono::milliseconds(static_cast<int64_t>(timeout * 1000.0));
  const auto connection = lower(get_parameter("connection_type").as_string());
  if (connection == "auto") {
    config.connection = NatNetConfig::Connection::AUTO;
  } else if (connection == "multicast") {
    config.connection = NatNetConfig::Connection::MULTICAST;
  } else if (connection == "unicast") {
    config.connection = NatNetConfig::Connection::UNICAST;
  } else {
    RCLCPP_ERROR(get_logger(), "connection_type must be auto, multicast or unicast");
    return CallbackReturn::FAILURE;
  }
  const auto up = lower(get_parameter("up_axis").as_string());
  if (up != "y" && up != "z") {
    RCLCPP_ERROR(get_logger(), "up_axis must be y or z (as set in Motive streaming settings)");
    return CallbackReturn::FAILURE;
  }
  options_.up_axis = up == "y" ? UpAxis::Y : UpAxis::Z;
  const auto skel = lower(get_parameter("skeleton_coordinates").as_string());
  if (skel != "global" && skel != "local") {
    RCLCPP_ERROR(get_logger(), "skeleton_coordinates must be global or local");
    return CallbackReturn::FAILURE;
  }
  options_.skeleton_local = skel == "local";
  compensate_latency_ = get_parameter("compensate_latency").as_bool();

  const auto backend = lower(get_parameter("backend").as_string());
  BackendKind kind;
  if (backend == "auto") {
    kind = BackendKind::AUTO;
  } else if (backend == "sdk") {
    kind = BackendKind::SDK;
  } else if (backend == "open") {
    kind = BackendKind::OPEN;
  } else {
    RCLCPP_ERROR(get_logger(), "backend must be auto, sdk or open");
    return CallbackReturn::FAILURE;
  }

  client_ = make_backend(kind, config);  // throws on invalid addresses or missing SDK
  server_ = client_->connect();          // throws if Motive does not answer
  server_label_ = server_.app_name + " " + version_string(server_.app_version) + " at " +
    config.server_address + " (NatNet " + version_string(server_.natnet_version) + ", " +
    (client_->multicast() ? "multicast" : "unicast") + ", " + client_->name() + " backend)";
  RCLCPP_INFO(
    get_logger(), "connected to %s %s (NatNet %s, %s, %s backend)", server_.app_name.c_str(),
    version_string(server_.app_version).c_str(), version_string(server_.natnet_version).c_str(),
    client_->multicast() ? "multicast" : "unicast", client_->name().c_str());

  const auto descriptions = client_->request_descriptions();
  {
    std::lock_guard<std::mutex> lock(model_mutex_);
    model_ = Model::from(descriptions);
  }
  RCLCPP_INFO(
    get_logger(), "%zu rigid bodies, %zu skeletons, %zu marker sets",
    descriptions.rigid_bodies.size(), descriptions.skeletons.size(),
    descriptions.marker_sets.size());

  publisher_ = std::make_unique<mocap4r2_control::FramePublisher>(*this);
  return CallbackReturn::SUCCESS;
}

OptitrackDriver::CallbackReturn
OptitrackDriver::on_system_activate()
{
  client_->start(
    [this](natnet::FrameData && frame) {on_frame(std::move(frame));},
    [this](const std::string & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.c_str());
    });
  return CallbackReturn::SUCCESS;
}

OptitrackDriver::CallbackReturn
OptitrackDriver::on_system_deactivate()
{
  if (client_) {
    client_->stop();
  }
  return CallbackReturn::SUCCESS;
}

OptitrackDriver::CallbackReturn
OptitrackDriver::on_system_cleanup()
{
  release_client();  // stops the threads and disconnects
  server_label_.clear();
  publisher_.reset();
  return CallbackReturn::SUCCESS;
}

void
OptitrackDriver::on_system_shutdown()
{
  release_client();
}

void
OptitrackDriver::refresh_model()
{
  // Assets were added or removed in Motive: reload names in the background.
  if (refreshing_.exchange(true)) {
    return;
  }
  if (refresh_thread_.joinable()) {
    refresh_thread_.join();  // previous refresh already finished
  }
  refresh_thread_ = std::thread(
    [this] {
      try {
        auto d = client_->request_descriptions();
        std::lock_guard<std::mutex> lock(model_mutex_);
        model_ = Model::from(d);
        RCLCPP_INFO(get_logger(), "asset list changed: %zu rigid bodies", d.rigid_bodies.size());
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "cannot reload data descriptions: %s", e.what());
      }
      refreshing_ = false;
    });
}

void
OptitrackDriver::on_frame(natnet::FrameData && frame)
{
  const auto received = now();
  if (frame.models_changed()) {
    refresh_model();
  }
  mocap4r2_control::MocapFrame out;
  {
    std::lock_guard<std::mutex> lock(model_mutex_);
    out = convert_frame(frame, model_, options_);
  }
  const double latency =
    compensate_latency_ ? frame_latency(frame, server_.clock_frequency) : 0.0;
  out.capture_time = received - rclcpp::Duration::from_seconds(latency);
  publisher_->publish(std::move(out));
}

}  // namespace mocap4r2_optitrack_driver

RCLCPP_COMPONENTS_REGISTER_NODE(mocap4r2_optitrack_driver::OptitrackDriver)
