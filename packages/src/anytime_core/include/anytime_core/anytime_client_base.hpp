#ifndef ANYTIME_CORE_ANYTIME_CLIENT_BASE_HPP
#define ANYTIME_CORE_ANYTIME_CLIENT_BASE_HPP

#include "anytime_core/tracing.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <functional>
#include <memory>
#include <string>

namespace anytime_core
{

/**
 * @brief Base class for Anytime action clients
 *
 * This template base class provides common functionality for both reactive and proactive
 * anytime action clients. It handles the common patterns of goal submission, feedback
 * processing, and result handling, while allowing derived classes to customize domain-specific
 * behavior through virtual functions.
 *
 * @tparam ActionType The ROS2 action type (e.g., anytime_interfaces::action::MonteCarlo)
 */
template <typename ActionType>
class AnytimeClientBase : public rclcpp::Node
{
public:
  using AnytimeGoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;

  explicit AnytimeClientBase(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node(node_name, options)
  {
    TRACE_ANYTIME_CLIENT_INIT(this, node_name.c_str());
    // Initialize the action client
    action_client_ = rclcpp_action::create_client<ActionType>(this, "anytime");
  }

  virtual ~AnytimeClientBase() = default;

protected:
  // ==================== Virtual Functions for Domain-Specific Behavior ====================

  /**
   * @brief Override this to define domain-specific post-processing after receiving a result
   *
   * This is called for both SUCCEEDED and CANCELED results. Derived classes should
   * implement any visualization, state cleanup, or other post-processing logic here.
   *
   * @param result The wrapped result from the action server
   */
  virtual void post_processing(const typename AnytimeGoalHandle::WrappedResult & result) = 0;

  /**
   * @brief Override this to define domain-specific result logging behavior
   *
   * Called when a result is successfully received. Derived classes can log
   * domain-specific information about the result.
   *
   * @param result The wrapped result from the action server
   */
  virtual void log_result(const typename AnytimeGoalHandle::WrappedResult & result)
  {
    (void)result;
    RCLCPP_DEBUG(
      this->get_logger(), "[Goal ID: %s] Result received",
      rclcpp_action::to_string(goal_handle_->get_goal_id()).c_str());
  }

  /**
   * @brief Override this to define domain-specific feedback logging
   *
   * Called when feedback is received from the action server.
   *
   * @param goal_handle The goal handle
   * @param feedback The feedback message
   */
  virtual void process_feedback(
    typename AnytimeGoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const typename ActionType::Feedback> feedback)
  {
    (void)goal_handle;
    (void)feedback;
    RCLCPP_DEBUG(
      this->get_logger(), "[Goal ID: %s] Feedback received",
      rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
  }

  /**
   * @brief Override this to perform cleanup after result is received
   *
   * This is called after post_processing and before resetting internal state.
   * Useful for domain-specific cleanup like resetting flags.
   */
  virtual void cleanup_after_result() {}

  // ==================== Common Result Callback Pattern ====================

  /**
   * @brief Common result callback implementation
   *
   * This implements the common pattern of handling action results. It:
   * 1. Logs the result based on the result code
   * 2. Calls post_processing for SUCCEEDED and CANCELED results
   * 3. Calls cleanup_after_result for cleanup
   */
  void result_callback(const typename AnytimeGoalHandle::WrappedResult & result)
  {
    TRACE_ANYTIME_CLIENT_RESULT(this, static_cast<int>(result.code));

    // Record goal finished timestamp
    auto goal_finished_time = this->now().nanoseconds();
    TRACE_ANYTIME_CLIENT_GOAL_FINISHED(this, goal_finished_time, static_cast<int>(result.code));

    // Log the result based on the result code
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        log_result(result);
        post_processing(result);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_DEBUG(
          this->get_logger(), "[Goal ID: %s] Goal was canceled",
          rclcpp_action::to_string(goal_handle_->get_goal_id()).c_str());
        post_processing(result);
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }

    // Perform cleanup
    cleanup_after_result();
  }

  /**
   * @brief Send a goal to the action server with standard callbacks
   *
   * This method sets up the standard goal response, feedback, and result callbacks,
   * waits for the server, and sends the goal asynchronously. This consolidates the
   * common goal-sending pattern used by both Monte Carlo and YOLO clients.
   *
   * @param goal_msg The goal message to send
   * @param on_server_unavailable Optional callback when server is unavailable (default: none)
   * @return true if goal was sent successfully, false otherwise
   */
  bool send_goal_to_server(
    const typename ActionType::Goal & goal_msg,
    std::function<void()> on_server_unavailable = nullptr)
  {
    TRACE_ANYTIME_CLIENT_SEND_GOAL(this);
    RCLCPP_DEBUG(this->get_logger(), "Sending goal to action server");

    // Define the goal options with callbacks
    auto send_goal_options = typename rclcpp_action::Client<ActionType>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](typename AnytimeGoalHandle::SharedPtr goal_handle) {
        this->goal_response_callback(goal_handle);
      };
    send_goal_options.feedback_callback =
      [this](
        typename AnytimeGoalHandle::SharedPtr goal_handle,
        const std::shared_ptr<const typename ActionType::Feedback> feedback) {
        this->feedback_callback(goal_handle, feedback);
      };
    send_goal_options.result_callback =
      [this](const typename AnytimeGoalHandle::WrappedResult & result) {
        this->result_callback(result);
      };

    // Wait for action server
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      if (on_server_unavailable) {
        on_server_unavailable();
      }
      return false;
    }

    // Send the goal asynchronously and record timestamp
    auto goal_sent_time = this->now().nanoseconds();
    TRACE_ANYTIME_CLIENT_GOAL_SENT(this, goal_sent_time);
    action_client_->async_send_goal(goal_msg, send_goal_options);
    return true;
  }

  /**
   * @brief Common goal response callback implementation
   *
   * Checks if the goal was rejected and stores the goal handle.
   */
  void goal_response_callback(typename AnytimeGoalHandle::SharedPtr goal_handle)
  {
    // Check if the goal was rejected by the server
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      TRACE_ANYTIME_CLIENT_GOAL_RESPONSE(this, false);
      on_goal_rejected();
      return;
    }

    // Store the goal handle for future reference
    goal_handle_ = goal_handle;

    RCLCPP_DEBUG(
      this->get_logger(), "[Goal ID: %s] Goal accepted by server",
      rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
    TRACE_ANYTIME_CLIENT_GOAL_RESPONSE(this, true);
    on_goal_accepted(goal_handle);
  }

  /**
   * @brief Common feedback callback that delegates to process_feedback
   */
  void feedback_callback(
    typename AnytimeGoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const typename ActionType::Feedback> feedback)
  {
    TRACE_ANYTIME_CLIENT_FEEDBACK(this);
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Feedback received for unknown goal handle");
      return;
    }
    RCLCPP_DEBUG(
      this->get_logger(), "[Goal ID: %s] Feedback callback triggered",
      rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
    process_feedback(goal_handle, feedback);
  }

  /**
   * @brief Override to handle goal rejection
   */
  virtual void on_goal_rejected() {}

  /**
   * @brief Override to handle goal acceptance
   *
   * @param goal_handle The accepted goal handle
   */
  virtual void on_goal_accepted(typename AnytimeGoalHandle::SharedPtr goal_handle)
  {
    (void)goal_handle;
  }

  // ==================== Protected Member Variables ====================

  /// Action client for the Anytime action
  typename rclcpp_action::Client<ActionType>::SharedPtr action_client_;

  /// Handle for the current goal
  typename AnytimeGoalHandle::SharedPtr goal_handle_ = nullptr;
};

}  // namespace anytime_core

#endif  // ANYTIME_CORE_ANYTIME_CLIENT_BASE_HPP
