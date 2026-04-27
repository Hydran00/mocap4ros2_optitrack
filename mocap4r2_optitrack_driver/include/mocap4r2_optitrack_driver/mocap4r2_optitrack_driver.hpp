// Copyright 2021 Institute for Robotics and Intelligent Machines,
//                Georgia Institute of Technology
// Copyright 2024 Intelligent Robotics Lab
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
//
// Author: Christian Llanes <christian.llanes@gatech.edu>
// Author: David Vargas Frutos <david.vargas@urjc.es>
// Author: Francisco Martín <fmrico@urjc.es>
// Copyright 2021 Institute for Robotics and Intelligent Machines,
//                Georgia Institute of Technology
// Copyright 2024 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at:
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing,
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors:
//   Christian Llanes <christian.llanes@gatech.edu>
//   David Vargas Frutos <david.vargas@urjc.es>
//   Francisco Martín <fmrico@urjc.es>

#ifndef MOCAP4R2_OPTITRACK_DRIVER__MOCAP4R2_OPTITRACK_DRIVER_HPP_
#define MOCAP4R2_OPTITRACK_DRIVER__MOCAP4R2_OPTITRACK_DRIVER_HPP_

#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/node_interfaces/node_logging.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/empty.hpp"

#include "mocap4r2_msgs/msg/marker.hpp"
#include "mocap4r2_msgs/msg/markers.hpp"
#include "mocap4r2_msgs/msg/rigid_body.hpp"
#include "mocap4r2_msgs/msg/rigid_bodies.hpp"
#include "mocap4r2_msgs/msg/skeleton.hpp"
#include "mocap4r2_msgs/msg/skeletons.hpp"

#include "tf2/buffer_core.h"
#include "tf2_ros/transform_broadcaster.h"

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>

#include "mocap4r2_control/ControlledLifecycleNode.hpp"

namespace mocap4r2_optitrack_driver
{

/// Lifecycle node for streaming motion capture data from OptiTrack via NatNet.
class OptitrackDriverNode : public mocap4r2_control::ControlledLifecycleNode
{
public:
  OptitrackDriverNode();
  ~OptitrackDriverNode();

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // --- Lifecycle callbacks ---
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state) override;
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state) override;

  // --- OptiTrack connection control ---
  bool connect_optitrack();
  bool disconnect_optitrack();
  void set_settings_optitrack();
  bool stop_optitrack();
  void initParameters();

  // --- Data processing ---
  void process_frame(sFrameOfMocapData * data);

protected:
  // --- Control service handlers ---
  void control_start(const mocap4r2_control_msgs::msg::Control::SharedPtr msg) override;
  void control_stop(const mocap4r2_control_msgs::msg::Control::SharedPtr msg) override;

  // --- NatNet client instance ---
  NatNetClient * client{nullptr};

  // --- Helper for system latency calculation ---
  std::chrono::nanoseconds get_optitrack_system_latency(sFrameOfMocapData * data);

  // --- NatNet state ---
  sNatNetClientConnectParams client_params;
  sServerDescription server_description;
  sDataDescriptions * data_descriptions{nullptr};
  sFrameOfMocapData latest_data;
  sRigidBodyData latest_body_frame_data;

  // --- Publishers ---
  rclcpp_lifecycle::LifecyclePublisher<mocap4r2_msgs::msg::Markers>::SharedPtr
    mocap4r2_markers_pub_;
  rclcpp_lifecycle::LifecyclePublisher<mocap4r2_msgs::msg::RigidBodies>::SharedPtr
    mocap4r2_rigid_body_pub_;
  rclcpp_lifecycle::LifecyclePublisher<mocap4r2_msgs::msg::Skeletons>::SharedPtr
    mocap4r2_skeleton_pub_;

  std::map<std::string, rclcpp_lifecycle::LifecyclePublisher<mocap4r2_msgs::msg::Markers>::SharedPtr>
    markerset_pubs_;
  mutable std::mutex markerset_pubs_mutex_;

  std::map<std::string, std::vector<std::string>> markerset_marker_names_;

  void build_markerset_name_map();

  // --- Connection parameters ---
  std::string connection_type_;
  std::string server_address_;
  std::string local_address_;
  std::string multicast_address_;
  // mapping from model ID to bone names
  std::map<int, std::string> skeleton_bone_names_; 
  uint16_t server_command_port_;
  uint16_t server_data_port_;

  // --- Frame counter ---
  uint32_t frame_number_{0};
};

/// Global callback trampoline for NatNet frame data.
void NATNET_CALLCONV process_frame_callback(sFrameOfMocapData * data, void * pUserData);

}  // namespace mocap4r2_optitrack_driver

#endif  // MOCAP4R2_OPTITRACK_DRIVER__MOCAP4R2_OPTITRACK_DRIVER_HPP_
