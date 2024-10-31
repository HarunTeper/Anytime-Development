#include "anytime_monte_carlo/anytime_client.hpp"

AnytimeActionClient::AnytimeActionClient(const rclcpp::NodeOptions& options)
    : Node("anytime_action_client", options) {
  RCLCPP_INFO(this->get_logger(), "Starting Anytime action client");

  // Initialize the action client
  action_client_ =
      rclcpp_action::create_client<anytime_interfaces::action::Anytime>(
          this, "anytime");

  // Create a timer to send goals periodically
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                   [this]() { this->send_goal(); });

  // Create a timer for cancel timeout, initially canceled
  cancel_timeout_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(50),
                              [this]() { this->cancel_timeout_callback(); });
  cancel_timeout_timer_->cancel();
}

AnytimeActionClient::~AnytimeActionClient() {}

void AnytimeActionClient::send_goal() {
  // Cancel the timer to prevent sending multiple goals
  timer_->cancel();
  // RCLCPP_INFO(this->get_logger(), "Sending goal");

  // Create and populate the goal message
  auto goal_msg = anytime_interfaces::action::Anytime::Goal();
  goal_msg.goal = 1000000;
  goal_msg.client_start = this->now();

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
  goal_msg.action_send = this->now();
  action_client_->async_send_goal(goal_msg, send_goal_options);
  // set the cancel time to the client start time
  cancel_time_ = goal_msg.client_start;
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
  // RCLCPP_INFO(this->get_logger(),
  //             "Goal accepted by server, waiting for result");

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
  receive_time_ = this->now();
  cancel_timeout_timer_->cancel();
  // Log the result based on the result code
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      // If the goal succeeded, log the result
      RCLCPP_INFO(this->get_logger(), "Result received: %f",
                  result.result->result);
      // print the number of iterations
      RCLCPP_INFO(this->get_logger(), "Number of iterations: %d",
                  result.result->iterations);
      print_time_differences(result);
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
      // print the number of iterations
      RCLCPP_INFO(this->get_logger(), "Number of iterations: %d",
                  result.result->iterations);
      print_time_differences(result);
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

  // Set the cancel time
  cancel_time_ = this->now();

  // Send a cancel request for the current goal
  action_client_->async_cancel_goal(goal_handle_);

  RCLCPP_INFO(this->get_logger(), "Cancel request sent");
}

void AnytimeActionClient::print_time_differences(
    const AnytimeGoalHandle::WrappedResult& result) {
  auto to_chrono = [](const builtin_interfaces::msg::Time& time) {
    return std::chrono::seconds(time.sec) +
           std::chrono::nanoseconds(time.nanosec);
  };

  auto client_start_chrono = to_chrono(result.result->client_start);
  auto action_send_chrono = to_chrono(result.result->action_send);
  auto action_accept_chrono = to_chrono(result.result->action_accept);
  auto action_start_chrono = to_chrono(result.result->action_start);
  auto action_cancel_chrono = to_chrono(cancel_time_);
  auto action_end_chrono = to_chrono(result.result->action_end);
  auto receive_chrono = to_chrono(receive_time_);

  auto diff_send = action_send_chrono - client_start_chrono;
  auto diff_accept = action_accept_chrono - client_start_chrono;
  auto diff_start = action_start_chrono - client_start_chrono;
  auto diff_cancel = action_cancel_chrono - client_start_chrono;
  auto diff_end = action_end_chrono - client_start_chrono;
  auto diff_receive = receive_chrono - client_start_chrono;
  auto total_computation = action_end_chrono - action_start_chrono;
  auto total_diff = receive_chrono - client_start_chrono;

  RCLCPP_INFO(this->get_logger(), "Time differences:");
  RCLCPP_INFO(this->get_logger(), "Send: %ld.%09ld seconds",
              diff_send.count() / 1000000000, diff_send.count() % 1000000000);
  RCLCPP_INFO(this->get_logger(), "Accept: %ld.%09ld seconds",
              diff_accept.count() / 1000000000,
              diff_accept.count() % 1000000000);
  RCLCPP_INFO(this->get_logger(), "Start: %ld.%09ld seconds",
              diff_start.count() / 1000000000, diff_start.count() % 1000000000);
  RCLCPP_INFO(this->get_logger(), "Cancel: %ld.%09ld seconds",
              diff_cancel.count() / 1000000000,
              diff_cancel.count() % 1000000000);
  RCLCPP_INFO(this->get_logger(), "End: %ld.%09ld seconds",
              diff_end.count() / 1000000000, diff_end.count() % 1000000000);
  RCLCPP_INFO(this->get_logger(), "Receive: %ld.%09ld seconds",
              diff_receive.count() / 1000000000,
              diff_receive.count() % 1000000000);
  RCLCPP_INFO(this->get_logger(), "Total computation: %ld.%09ld seconds",
              total_computation.count() / 1000000000,
              total_computation.count() % 1000000000);
  RCLCPP_INFO(this->get_logger(), "Total: %ld.%09ld seconds",
              total_diff.count() / 1000000000, total_diff.count() % 1000000000);
}