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

#include "anytime_monte_carlo/anytime_server.hpp"

#include "rclcpp/logging.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// Constructor for the AnytimeActionServer class
AnytimeActionServer::AnytimeActionServer(rclcpp::NodeOptions options)
: anytime_core::AnytimeActionServerBase<Anytime>(
    "anytime_action_server", options.use_intra_process_comms(true))
{
  RCLCPP_DEBUG(this->get_logger(), "Starting Anytime action server");

  // Read the ros2 parameters
  std::string reactive_proactive_str = this->declare_parameter("is_reactive_proactive", "reactive");
  int block_size = this->declare_parameter("block_size", 1);

  // Convert strings to booleans
  bool is_reactive_proactive = (reactive_proactive_str == "proactive");

  RCLCPP_INFO(this->get_logger(), "Monte Carlo Action Server initialized with parameters:");
  RCLCPP_INFO(this->get_logger(), "  is_reactive_proactive: %s", reactive_proactive_str.c_str());
  RCLCPP_INFO(this->get_logger(), "  block_size: %d", block_size);

  RCLCPP_DEBUG(
    this->get_logger(), "is_reactive_proactive: %s",
    is_reactive_proactive ? "proactive" : "reactive");
  RCLCPP_DEBUG(this->get_logger(), "block_size: %d", block_size);

  // Create the Anytime management object based on the parameters
  this->anytime_management_ = create_anytime_management(this, is_reactive_proactive, block_size);
}

// Destructor for the AnytimeActionServer class
AnytimeActionServer::~AnytimeActionServer() {}

// factory function
std::shared_ptr<
  anytime_core::AnytimeBase<AnytimeActionServer::Anytime, AnytimeActionServer::GoalHandleType>>
AnytimeActionServer::create_anytime_management(
  rclcpp::Node * node, bool is_reactive_proactive, int block_size)
{
  if (is_reactive_proactive) {
    return std::make_shared<AnytimeManagement<true>>(node, block_size);
  } else {
    return std::make_shared<AnytimeManagement<false>>(node, block_size);
  }
}

// Register the component
RCLCPP_COMPONENTS_REGISTER_NODE(AnytimeActionServer)
