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
// Author: David Vargas Frutos <david.vargas@urjc.es>
// Author: Francisco Martín <fmrico@urjc.es>

#include <iostream>
#include <memory>
#include "mocap4r2_optitrack_driver/mocap4r2_optitrack_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mocap4r2_optitrack_driver::OptitrackDriverNode>();

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node->get_node_base_interface());

  rclcpp::TimerBase::SharedPtr timer;
  timer = node->create_wall_timer(
    std::chrono::seconds(1),
    [&node, &timer]() {
      timer->cancel();
      node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    });

  executor->spin();
  rclcpp::shutdown();
  return 0;
}
