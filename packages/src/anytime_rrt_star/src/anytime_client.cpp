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

#include "anytime_rrt_star/anytime_client.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

AnytimeRrtActionClient::AnytimeRrtActionClient(const rclcpp::NodeOptions & options)
: anytime_core::AnytimeClientBase<Anytime>("anytime_action_client", options)
{
  RCLCPP_DEBUG(this->get_logger(), "Starting RRT* Anytime action client");

  this->declare_parameter("goal_timer_period_ms", 100);
  this->declare_parameter("cancel_timeout_period_ms", 50);

  int goal_timer_period = this->get_parameter("goal_timer_period_ms").as_int();
  int cancel_timeout_period = this->get_parameter("cancel_timeout_period_ms").as_int();

  RCLCPP_INFO(this->get_logger(), "RRT* Action Client initialized with parameters:");
  RCLCPP_INFO(this->get_logger(), "  goal_timer_period_ms: %d", goal_timer_period);
  RCLCPP_INFO(this->get_logger(), "  cancel_timeout_period_ms: %d", cancel_timeout_period);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(goal_timer_period), [this]() {this->send_goal();});

  cancel_timeout_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(cancel_timeout_period),
    [this]() {this->cancel_timeout_callback();});
  cancel_timeout_timer_->cancel();
}

AnytimeRrtActionClient::~AnytimeRrtActionClient() {}

void AnytimeRrtActionClient::send_goal()
{
  RCLCPP_DEBUG(this->get_logger(), "Sending RRT* goal");
  timer_->cancel();

  auto goal_msg = Anytime::Goal();
  goal_msg.goal = 100000;  // Default: 100k RRT* iterations

  send_goal_to_server(goal_msg, [this]() {timer_->reset();});
}

void AnytimeRrtActionClient::on_goal_rejected()
{
  timer_->reset();
}

void AnytimeRrtActionClient::on_goal_accepted(AnytimeGoalHandle::SharedPtr goal_handle)
{
  RCLCPP_DEBUG(
    this->get_logger(), "[Goal ID: %s] Goal accepted, starting cancel timeout timer",
    rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
  cancel_timeout_timer_->reset();
}

void AnytimeRrtActionClient::process_feedback(
  AnytimeGoalHandle::SharedPtr goal_handle, const std::shared_ptr<const Anytime::Feedback> feedback)
{
  RCLCPP_DEBUG(
    this->get_logger(), "[Goal ID: %s] Current best path cost: %f",
    rclcpp_action::to_string(goal_handle->get_goal_id()).c_str(), feedback->feedback);
}

void AnytimeRrtActionClient::log_result(const AnytimeGoalHandle::WrappedResult & result)
{
  RCLCPP_DEBUG(
    this->get_logger(), "[Goal ID: %s] Best path cost: %f",
    rclcpp_action::to_string(goal_handle_->get_goal_id()).c_str(), result.result->result);
  RCLCPP_DEBUG(
    this->get_logger(), "[Goal ID: %s] Iterations: %d, Tree size: %d, First solution iter: %d",
    rclcpp_action::to_string(goal_handle_->get_goal_id()).c_str(),
    result.result->iterations, result.result->tree_size, result.result->first_solution_iteration);
}

void AnytimeRrtActionClient::post_processing(const AnytimeGoalHandle::WrappedResult & result)
{
  (void)result;
}

void AnytimeRrtActionClient::cleanup_after_result()
{
  cancel_timeout_timer_->cancel();
  goal_handle_.reset();
  timer_->reset();
}

void AnytimeRrtActionClient::cancel_timeout_callback()
{
  if (!goal_handle_) {
    RCLCPP_WARN(this->get_logger(), "No active goal to cancel");
    return;
  }

  RCLCPP_DEBUG(
    this->get_logger(), "[Goal ID: %s] Timeout reached, sending cancel request",
    rclcpp_action::to_string(goal_handle_->get_goal_id()).c_str());

  cancel_timeout_timer_->cancel();

  try {
    auto cancel_sent_time = this->now().nanoseconds();
    TRACE_ANYTIME_CLIENT_CANCEL_SENT(this, cancel_sent_time);

    auto cancel_future = action_client_->async_cancel_goal(goal_handle_);
    RCLCPP_DEBUG(
      this->get_logger(), "[Goal ID: %s] Cancel request sent",
      rclcpp_action::to_string(goal_handle_->get_goal_id()).c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "[Goal ID: %s] Failed to send cancel request: %s",
      rclcpp_action::to_string(goal_handle_->get_goal_id()).c_str(), e.what());
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(AnytimeRrtActionClient)
