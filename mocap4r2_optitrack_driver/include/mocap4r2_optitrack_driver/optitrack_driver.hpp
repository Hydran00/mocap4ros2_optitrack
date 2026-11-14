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

#ifndef MOCAP4R2_OPTITRACK_DRIVER__OPTITRACK_DRIVER_HPP_
#define MOCAP4R2_OPTITRACK_DRIVER__OPTITRACK_DRIVER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "mocap4r2_control/frame_publisher.hpp"
#include "mocap4r2_control/mocap_system_node.hpp"
#include "mocap4r2_optitrack_driver/frame_converter.hpp"
#include "mocap4r2_optitrack_driver/natnet_backend.hpp"

namespace mocap4r2_optitrack_driver
{

// MOCAP4ROS2 driver of OptiTrack systems (Motive), through the official NatNet
// SDK when the package was built with it, or the open NatNet implementation.
class OptitrackDriver : public mocap4r2_control::MocapSystemNode
{
public:
  explicit OptitrackDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~OptitrackDriver() override;

protected:
  CallbackReturn on_system_configure() override;
  CallbackReturn on_system_activate() override;
  CallbackReturn on_system_deactivate() override;
  CallbackReturn on_system_cleanup() override;
  void on_system_shutdown() override;
  std::string server() const override {return server_label_;}
  bool connected() const override {return client_ != nullptr;}

private:
  void on_frame(natnet::FrameData && frame);
  void refresh_model();
  void release_client();

  std::unique_ptr<NatNetBackend> client_;
  std::unique_ptr<mocap4r2_control::FramePublisher> publisher_;
  natnet::ServerInfo server_;
  ConverterOptions options_;
  bool compensate_latency_{true};
  std::string server_label_;

  std::mutex model_mutex_;
  Model model_;
  std::atomic<bool> refreshing_{false};
  std::thread refresh_thread_;
};

}  // namespace mocap4r2_optitrack_driver

#endif  // MOCAP4R2_OPTITRACK_DRIVER__OPTITRACK_DRIVER_HPP_
