// Copyright 2025 Anytime System
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

#include "anytime_rrt_star/anytime_server.hpp"

#include "rclcpp/logging.hpp"
#include "rclcpp_components/register_node_macro.hpp"

AnytimeRrtActionServer::AnytimeRrtActionServer(rclcpp::NodeOptions options)
: anytime_core::AnytimeActionServerBase<Anytime>(
    "anytime_action_server", options.use_intra_process_comms(true))
{
  RCLCPP_DEBUG(this->get_logger(), "Starting RRT* Anytime action server");

  std::string reactive_proactive_str = this->declare_parameter("is_reactive_proactive", "reactive");
  int batch_size = this->declare_parameter("batch_size", 1);

  // Declare RRT* algorithm parameters (read by AnytimeManagement constructor)
  this->declare_parameter("map_yaml_path", "");
  this->declare_parameter("start_x", 5.0);
  this->declare_parameter("start_y", 5.0);
  this->declare_parameter("goal_x", 25.0);
  this->declare_parameter("goal_y", 10.0);
  this->declare_parameter("step_size", 0.5);
  this->declare_parameter("goal_threshold", 0.5);
  this->declare_parameter("goal_bias", 0.05);
  this->declare_parameter("gamma_rrt_star", 0.0);  // 0 = auto-compute from map
  this->declare_parameter("prune_interval", 1000);

  bool is_reactive_proactive = (reactive_proactive_str == "proactive");

  RCLCPP_INFO(this->get_logger(), "RRT* Action Server initialized with parameters:");
  RCLCPP_INFO(this->get_logger(), "  is_reactive_proactive: %s", reactive_proactive_str.c_str());
  RCLCPP_INFO(this->get_logger(), "  batch_size: %d", batch_size);

  this->anytime_management_ = create_anytime_management(this, is_reactive_proactive, batch_size);
}

AnytimeRrtActionServer::~AnytimeRrtActionServer() {}

std::shared_ptr<
  anytime_core::AnytimeBase<AnytimeRrtActionServer::Anytime, AnytimeRrtActionServer::GoalHandleType>>
AnytimeRrtActionServer::create_anytime_management(
  rclcpp::Node * node, bool is_reactive_proactive, int batch_size)
{
  if (is_reactive_proactive) {
    return std::make_shared<AnytimeManagement<true>>(node, batch_size);
  } else {
    return std::make_shared<AnytimeManagement<false>>(node, batch_size);
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(AnytimeRrtActionServer)
