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

#ifndef ANYTIME_RRT_STAR__ANYTIME_SERVER_HPP_
#define ANYTIME_RRT_STAR__ANYTIME_SERVER_HPP_

#include <memory>

#include "anytime_core/anytime_server.hpp"
#include "anytime_interfaces/action/rrt_star.hpp"
#include "anytime_rrt_star/anytime_management.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class AnytimeRrtActionServer
  : public anytime_core::AnytimeActionServerBase<anytime_interfaces::action::RrtStar>
{
public:
  using Anytime = anytime_interfaces::action::RrtStar;
  using GoalHandleType = rclcpp_action::ServerGoalHandle<Anytime>;

  explicit AnytimeRrtActionServer(rclcpp::NodeOptions options = rclcpp::NodeOptions());
  ~AnytimeRrtActionServer() override;

private:
  std::shared_ptr<anytime_core::AnytimeBase<Anytime, GoalHandleType>> create_anytime_management(
    rclcpp::Node * node, bool is_reactive_proactive, int block_size);
};

#endif  // ANYTIME_RRT_STAR__ANYTIME_SERVER_HPP_
