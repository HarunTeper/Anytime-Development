#include "anytime_monte_carlo/anytime_client.hpp"

AnytimeActionClient::AnytimeActionClient(const rclcpp::NodeOptions& options)
    : Node("anytime_action_client", options) {
  RCLCPP_INFO(this->get_logger(), "Starting Anytime action client");

  // Initialize the action client
  action_client_ =
      rclcpp_action::create_client<anytime_interfaces::action::Anytime>(
          this, "anytime");

  // Create a timer to send goals periodically
  timer_ = this->create_wall_timer(std::chrono::seconds(2),
                                   [this]() { this->send_goal(); });

  // Create a timer for cancel timeout, initially canceled
  cancel_timeout_timer_ = this->create_wall_timer(
      std::chrono::seconds(1), [this]() { this->cancel_timeout_callback(); });
  cancel_timeout_timer_->cancel();
}

AnytimeActionClient::~AnytimeActionClient() {}

void AnytimeActionClient::send_goal() {
  // Cancel the timer to prevent sending multiple goals
  timer_->cancel();
  RCLCPP_INFO(this->get_logger(), "Sending goal");

  // Create and populate the goal message
  auto goal_msg = anytime_interfaces::action::Anytime::Goal();
  goal_msg.goal = 5000000;

  // Define the goal options with callbacks
  auto send_goal_options = rclcpp_action::Client<
      anytime_interfaces::action::Anytime>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      [this](AnytimeGoalHandle::SharedPtr goal_handle) {
        this->goal_response_callback(goal_handle);
      };
  send_goal_options.feedback_callback =
      [this](AnytimeGoalHandle::SharedPtr goal_handle,
             const std::shared_ptr<
                 const anytime_interfaces::action::Anytime::Feedback>
                 feedback) { this->feedback_callback(goal_handle, feedback); };
  send_goal_options.result_callback =
      [this](const AnytimeGoalHandle::WrappedResult& result) {
        this->result_callback(result);
      };

  // Send the goal asynchronously
  action_client_->async_send_goal(goal_msg, send_goal_options);
}

void AnytimeActionClient::goal_response_callback(
    AnytimeGoalHandle::SharedPtr goal_handle) {
  // Check if the goal was rejected by the server
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    // Reset the timer to try sending the goal again
    timer_->reset();
    return;
  }

  // Log that the goal was accepted by the server
  RCLCPP_INFO(this->get_logger(),
              "Goal accepted by server, waiting for result");

  // Store the goal handle for future reference
  goal_handle_ = goal_handle;

  // Reset the cancel timeout timer to start counting down
  cancel_timeout_timer_->reset();
}

void AnytimeActionClient::feedback_callback(
    AnytimeGoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const anytime_interfaces::action::Anytime::Feedback>
        feedback) {
  (void)goal_handle;
  // Log the feedback received from the action server
  RCLCPP_INFO(this->get_logger(), "Next number in the sequence: %f",
              feedback->feedback);
}

void AnytimeActionClient::result_callback(
    const AnytimeGoalHandle::WrappedResult& result) {
  cancel_timeout_timer_->cancel();
  // Log the result based on the result code
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      // If the goal succeeded, log the result
      RCLCPP_INFO(this->get_logger(), "Result received: %f",
                  result.result->result);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      // If the goal was aborted, log an error
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      // If the goal was canceled, log an error and the result after cancel
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      RCLCPP_INFO(this->get_logger(), "Result after cancel callback: %f",
                  result.result->result);
      break;
    default:
      // If the result code is unknown, log an error
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
  }

  // Reset the timer to allow sending new goals
  timer_->reset();
}

void AnytimeActionClient::cancel_timeout_callback() {
  RCLCPP_INFO(this->get_logger(), "Timeout reached, canceling goal");

  // Cancel the timeout timer to prevent multiple cancel requests
  cancel_timeout_timer_->cancel();

  // Send a cancel request for the current goal
  action_client_->async_cancel_goal(goal_handle_);

  RCLCPP_INFO(this->get_logger(), "Cancel request sent");
}