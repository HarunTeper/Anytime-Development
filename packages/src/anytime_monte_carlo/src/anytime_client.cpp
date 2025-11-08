#include "anytime_monte_carlo/anytime_client.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <vector>

AnytimeActionClient::AnytimeActionClient(const rclcpp::NodeOptions & options)
: anytime_core::AnytimeClientBase<Anytime>("anytime_action_client", options)
{
  RCLCPP_DEBUG(this->get_logger(), "Starting Anytime action client");

  // Declare domain-specific parameters with default values
  this->declare_parameter("goal_timer_period_ms", 100);
  this->declare_parameter("cancel_timeout_period_ms", 50);

  int goal_timer_period = this->get_parameter("goal_timer_period_ms").as_int();
  int cancel_timeout_period = this->get_parameter("cancel_timeout_period_ms").as_int();

  RCLCPP_INFO(this->get_logger(), "Monte Carlo Action Client initialized with parameters:");
  RCLCPP_INFO(this->get_logger(), "  goal_timer_period_ms: %d", goal_timer_period);
  RCLCPP_INFO(this->get_logger(), "  cancel_timeout_period_ms: %d", cancel_timeout_period);

  RCLCPP_DEBUG(this->get_logger(), "Goal timer period: %d ms", goal_timer_period);
  RCLCPP_DEBUG(this->get_logger(), "Cancel timeout period: %d ms", cancel_timeout_period);

  RCLCPP_DEBUG(this->get_logger(), "Anytime action client created");

  // Create a timer to send goals periodically
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(goal_timer_period), [this]() { this->send_goal(); });

  RCLCPP_DEBUG(this->get_logger(), "Anytime action client initialized");

  // Create a timer for cancel timeout, initially canceled
  cancel_timeout_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(cancel_timeout_period),
    [this]() { this->cancel_timeout_callback(); });

  RCLCPP_DEBUG(this->get_logger(), "Anytime action client created");
  cancel_timeout_timer_->cancel();

  RCLCPP_DEBUG(this->get_logger(), "Anytime action client initialized");
}

AnytimeActionClient::~AnytimeActionClient() {}

void AnytimeActionClient::send_goal()
{
  RCLCPP_DEBUG(this->get_logger(), "Sending goal info");
  // Cancel the timer to prevent sending multiple goals
  timer_->cancel();

  // Create and populate the goal message
  auto goal_msg = Anytime::Goal();
  goal_msg.goal = 100000000;

  // Use the base class helper to send the goal
  send_goal_to_server(goal_msg, [this]() { timer_->reset(); });
}

void AnytimeActionClient::on_goal_rejected()
{
  // Reset the timer to try sending the goal again
  timer_->reset();
}

void AnytimeActionClient::on_goal_accepted(AnytimeGoalHandle::SharedPtr goal_handle)
{
  RCLCPP_DEBUG(
    this->get_logger(), "[Goal ID: %s] Goal accepted, starting cancel timeout timer",
    rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());

  // Reset the cancel timeout timer to start counting down
  cancel_timeout_timer_->reset();
}

void AnytimeActionClient::process_feedback(
  AnytimeGoalHandle::SharedPtr goal_handle, const std::shared_ptr<const Anytime::Feedback> feedback)
{
  // Log the feedback received from the action server
  RCLCPP_DEBUG(
    this->get_logger(), "[Goal ID: %s] Next number in the sequence: %f",
    rclcpp_action::to_string(goal_handle->get_goal_id()).c_str(), feedback->feedback);
}

void AnytimeActionClient::log_result(const AnytimeGoalHandle::WrappedResult & result)
{
  // Log domain-specific result information
  RCLCPP_DEBUG(
    this->get_logger(), "[Goal ID: %s] Result received: %f",
    rclcpp_action::to_string(goal_handle_->get_goal_id()).c_str(), result.result->result);
  RCLCPP_DEBUG(
    this->get_logger(), "[Goal ID: %s] Number of iterations: %d",
    rclcpp_action::to_string(goal_handle_->get_goal_id()).c_str(), result.result->iterations);
}

void AnytimeActionClient::post_processing(const AnytimeGoalHandle::WrappedResult & result)
{
  (void)result;
  // Monte Carlo doesn't need any post-processing like YOLO
  // The main result has already been logged in log_result
}

void AnytimeActionClient::cleanup_after_result()
{
  // Cancel the timeout timer if it's still running
  cancel_timeout_timer_->cancel();

  // Clear the goal handle to prevent stale references
  goal_handle_.reset();

  // Reset the timer to allow sending new goals
  timer_->reset();
}

void AnytimeActionClient::cancel_timeout_callback()
{
  // Check if goal_handle_ is valid before attempting to cancel
  if (!goal_handle_) {
    RCLCPP_WARN(this->get_logger(), "No active goal to cancel");
    return;
  }

  RCLCPP_DEBUG(
    this->get_logger(), "[Goal ID: %s] Timeout reached, canceling timer",
    rclcpp_action::to_string(goal_handle_->get_goal_id()).c_str());

  // Cancel the timeout timer to prevent multiple cancel requests
  cancel_timeout_timer_->cancel();

  RCLCPP_DEBUG(
    this->get_logger(), "[Goal ID: %s] Sending cancel request",
    rclcpp_action::to_string(goal_handle_->get_goal_id()).c_str());

  // Send a cancel request for the current goal
  try {
    // Record cancel sent timestamp
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

// Register the component
RCLCPP_COMPONENTS_REGISTER_NODE(AnytimeActionClient)