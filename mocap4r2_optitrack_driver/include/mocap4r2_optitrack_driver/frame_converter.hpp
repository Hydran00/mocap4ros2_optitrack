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

#ifndef MOCAP4R2_OPTITRACK_DRIVER__FRAME_CONVERTER_HPP_
#define MOCAP4R2_OPTITRACK_DRIVER__FRAME_CONVERTER_HPP_

#include <map>
#include <string>
#include <vector>

#include "mocap4r2_control/frame_publisher.hpp"
#include "mocap4r2_optitrack_driver/natnet_protocol.hpp"

namespace mocap4r2_optitrack_driver
{

// Names and structure of the assets, from the data descriptions.
struct Model
{
  std::map<int32_t, std::string> asset_names;                 // rigid bodies and skeletons
  std::map<std::string, std::vector<std::string>> marker_names;  // by asset name
  std::map<int32_t, natnet::SkeletonDescription> skeletons;

  static Model from(const natnet::DataDescriptions & d);
};

enum class UpAxis {Y, Z};

struct ConverterOptions
{
  UpAxis up_axis{UpAxis::Y};   // Motive streams Y-up unless configured otherwise
  bool skeleton_local{false};  // bones streamed in local (parent) coordinates
};

// Convert a NatNet frame to a MOCAP4ROS2 frame in a Z-up world (REP-103).
// capture_time is left for the caller.
mocap4r2_control::MocapFrame convert_frame(
  const natnet::FrameData & frame, const Model & model, const ConverterOptions & options);

// Motive latency of a frame (mid exposure to transmit), seconds; 0 if unknown.
double frame_latency(const natnet::FrameData & frame, uint64_t clock_frequency);

}  // namespace mocap4r2_optitrack_driver

#endif  // MOCAP4R2_OPTITRACK_DRIVER__FRAME_CONVERTER_HPP_
