#include "anytime_monte_carlo/anytime_client.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

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
  goal_msg.goal = 10000000;
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
  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
    timer_->reset();
    return;
  }
  action_client_->async_send_goal(goal_msg, send_goal_options);
  // set the cancel time to the client start time
  cancel_send_time_ = goal_msg.client_start;
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
  cancel_send_time_ = this->now();

  // Send a cancel request for the current goal
  action_client_->async_cancel_goal(goal_handle_);

  RCLCPP_INFO(this->get_logger(), "Cancel request sent");
}

void AnytimeActionClient::print_time_differences(
    const AnytimeGoalHandle::WrappedResult& result) {
  auto action_send_time = rclcpp::Time(result.result->action_send);
  auto action_start_time = rclcpp::Time(result.result->action_start);
  auto action_cancel_time = rclcpp::Time(result.result->action_cancel);
  auto action_end_time = rclcpp::Time(result.result->action_end);
  auto receive_time = receive_time_;

  auto interval1 = action_start_time - action_send_time;
  auto interval2 = action_cancel_time - cancel_send_time_;
  auto interval3 = action_end_time - action_cancel_time;
  auto interval4 = receive_time - action_end_time;
  auto interval5 = receive_time - cancel_send_time_;
  auto interval6 = receive_time - action_send_time;

  intervals1_.push_back(interval1.nanoseconds());
  intervals2_.push_back(interval2.nanoseconds());
  intervals3_.push_back(interval3.nanoseconds());
  intervals4_.push_back(interval4.nanoseconds());
  intervals5_.push_back(interval5.nanoseconds());
  intervals6_.push_back(interval6.nanoseconds());

  auto calculate_stats = [](const std::vector<int64_t>& intervals) {
    double mean = std::accumulate(intervals.begin(), intervals.end(), 0.0) /
                  intervals.size() / 1e6;  // Convert to milliseconds
    double sq_sum = std::inner_product(intervals.begin(), intervals.end(),
                                       intervals.begin(), 0.0) /
                    1e12;  // Convert to milliseconds^2
    double stdev = std::sqrt(sq_sum / intervals.size() - mean * mean);
    auto max = *std::max_element(intervals.begin(), intervals.end()) /
               1e6;  // Convert to milliseconds
    auto p99 = intervals[static_cast<size_t>(intervals.size() * 0.99)] /
               1e6;  // Convert to milliseconds
    auto p999 = intervals[static_cast<size_t>(intervals.size() * 0.999)] /
                1e6;  // Convert to milliseconds
    return std::make_tuple(mean, stdev, p99, p999, max);
  };

  auto [mean1, stdev1, p99_1, p999_1, max1] = calculate_stats(intervals1_);
  auto [mean2, stdev2, p99_2, p999_2, max2] = calculate_stats(intervals2_);
  auto [mean3, stdev3, p99_3, p999_3, max3] = calculate_stats(intervals3_);
  auto [mean4, stdev4, p99_4, p999_4, max4] = calculate_stats(intervals4_);
  auto [mean5, stdev5, p99_5, p999_5, max5] = calculate_stats(intervals5_);
  auto [mean6, stdev6, p99_6, p999_6, max6] = calculate_stats(intervals6_);

  RCLCPP_INFO(
      this->get_logger(),
      "Interval 1 (action_send to action_start): mean=%f ms, stdev=%f ms, "
      "p99=%f ms, p999=%f ms, max=%f ms",
      mean1, stdev1, p99_1, p999_1, max1);
  RCLCPP_INFO(this->get_logger(),
              "Interval 2 (cancel_send_time_ to action_cancel): mean=%f ms, "
              "stdev=%f ms, "
              "p99=%f ms, p999=%f ms, max=%f ms",
              mean2, stdev2, p99_2, p999_2, max2);
  RCLCPP_INFO(
      this->get_logger(),
      "Interval 3 (action_cancel to action_end): mean=%f ms, stdev=%f ms, "
      "p99=%f ms, p999=%f ms, max=%f ms",
      mean3, stdev3, p99_3, p999_3, max3);
  RCLCPP_INFO(this->get_logger(),
              "Interval 4 (action_end to receive): mean=%f ms, stdev=%f ms, "
              "p99=%f ms, p999=%f ms, max=%f ms",
              mean4, stdev4, p99_4, p999_4, max4);
  RCLCPP_INFO(this->get_logger(),
              "Interval 5 (action_cancel to receive): mean=%f ms, stdev=%f ms, "
              "p99=%f ms, p999=%f ms, max=%f ms",
              mean5, stdev5, p99_5, p999_5, max5);
  RCLCPP_INFO(this->get_logger(),
              "Interval 6 (action_send to receive): mean=%f ms, stdev=%f ms, "
              "p99=%f ms, p999=%f ms, max=%f ms",
              mean6, stdev6, p99_6, p999_6, max6);

  // Calculate the statistics for the iterations, which is an integer
  auto iterations = result.result->iterations;
  iterations_.push_back(iterations);

  auto mean = std::accumulate(iterations_.begin(), iterations_.end(), 0.0) /
              iterations_.size();
  auto sq_sum = std::inner_product(iterations_.begin(), iterations_.end(),
                                   iterations_.begin(), 0.0);
  auto stdev = std::sqrt(sq_sum / iterations_.size() - mean * mean);
  auto max = *std::max_element(iterations_.begin(), iterations_.end());
  auto p99 = iterations_[static_cast<size_t>(iterations_.size() * 0.99)];
  auto p999 = iterations_[static_cast<size_t>(iterations_.size() * 0.999)];

  RCLCPP_INFO(
      this->get_logger(),
      "Number of iterations: mean=%f, stdev=%f, p99=%li, p999=%li, max=%li",
      mean, stdev, p99, p999, max);
}