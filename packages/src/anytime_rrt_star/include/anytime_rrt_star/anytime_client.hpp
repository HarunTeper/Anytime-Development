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

#ifndef ANYTIME_RRT_STAR__ANYTIME_CLIENT_HPP_
#define ANYTIME_RRT_STAR__ANYTIME_CLIENT_HPP_

#include <memory>

#include "anytime_core/anytime_client_base.hpp"
#include "anytime_interfaces/action/rrt_star.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class AnytimeRrtActionClient
  : public anytime_core::AnytimeClientBase<anytime_interfaces::action::RrtStar>
{
public:
  using Anytime = anytime_interfaces::action::RrtStar;
  using AnytimeGoalHandle = rclcpp_action::ClientGoalHandle<Anytime>;

  explicit AnytimeRrtActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AnytimeRrtActionClient();

private:
  rclcpp::TimerBase::SharedPtr timer_ = nullptr;
  rclcpp::TimerBase::SharedPtr cancel_timeout_timer_ = nullptr;

  void send_goal();
  void cancel_timeout_callback();

  void post_processing(const AnytimeGoalHandle::WrappedResult & result) override;
  void log_result(const AnytimeGoalHandle::WrappedResult & result) override;
  void process_feedback(
    AnytimeGoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const Anytime::Feedback> feedback) override;
  void on_goal_rejected() override;
  void on_goal_accepted(AnytimeGoalHandle::SharedPtr goal_handle) override;
  void cleanup_after_result() override;
};

#endif  // ANYTIME_RRT_STAR__ANYTIME_CLIENT_HPP_
