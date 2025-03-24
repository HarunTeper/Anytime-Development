#include "anytime_yolo/anytime_client.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <vector>

AnytimeActionClient::AnytimeActionClient(const rclcpp::NodeOptions & options)
: Node("anytime_action_client", options)
{
  RCLCPP_DEBUG(this->get_logger(), "Starting Anytime action client");

  // Declare parameters with default values
  this->declare_parameter("goal_timer_period_ms", 1000);
  this->declare_parameter("cancel_timeout_period_ms", 500);
  this->declare_parameter("result_filename", "anytime_results");
  this->declare_parameter("image_topic", "video_frames");

  int goal_timer_period = this->get_parameter("goal_timer_period_ms").as_int();
  int cancel_timeout_period = this->get_parameter("cancel_timeout_period_ms").as_int();
  result_filename_ = this->get_parameter("result_filename").as_string();
  std::string image_topic = this->get_parameter("image_topic").as_string();

  RCLCPP_DEBUG(this->get_logger(), "Goal timer period: %d ms", goal_timer_period);
  RCLCPP_DEBUG(this->get_logger(), "Cancel timeout period: %d ms", cancel_timeout_period);
  RCLCPP_DEBUG(this->get_logger(), "Result filename: %s", result_filename_.c_str());
  RCLCPP_DEBUG(this->get_logger(), "Image topic: %s", image_topic.c_str());

  // Initialize the image subscription
  image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic, 10,
    [this](const sensor_msgs::msg::Image::SharedPtr msg) { this->image_callback(msg); });

  // Create a timer for cancel timeout, initially canceled
  cancel_timeout_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(cancel_timeout_period),
    [this]() { this->cancel_timeout_callback(); });
  cancel_timeout_timer_->cancel();

  // Initialize the action client
  action_client_ = rclcpp_action::create_client<Anytime>(this, "anytime");
}

AnytimeActionClient::~AnytimeActionClient() {}

// Fill the image callback implementation with image data
void AnytimeActionClient::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  client_goal_start_time_ = this->now();
  Anytime::Goal goal_msg;

  // Fill in the goal message with data from the image message
  goal_msg.data = msg->data;
  goal_msg.height = msg->height;
  goal_msg.width = msg->width;
  goal_msg.channels = msg->step / msg->width;  // Calculate channels from step and width
  goal_msg.encoding = msg->encoding;

  send_goal(goal_msg);
}

void AnytimeActionClient::send_goal(const Anytime::Goal & goal_msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Sending goal info");

  // Define the goal options with callbacks
  auto send_goal_options = rclcpp_action::Client<Anytime>::SendGoalOptions();
  send_goal_options.goal_response_callback = [this](AnytimeGoalHandle::SharedPtr goal_handle) {
    this->goal_response_callback(goal_handle);
  };
  send_goal_options.feedback_callback = [this](
                                          AnytimeGoalHandle::SharedPtr goal_handle,
                                          const std::shared_ptr<const Anytime::Feedback> feedback) {
    this->feedback_callback(goal_handle, feedback);
  };
  send_goal_options.result_callback = [this](const AnytimeGoalHandle::WrappedResult & result) {
    this->result_callback(result);
  };

  // Send the goal asynchronously
  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    return;
  }

  client_send_start_time_ = this->now();

  action_client_->async_send_goal(goal_msg, send_goal_options);

  client_send_end_time_ = this->now();
}

void AnytimeActionClient::goal_response_callback(AnytimeGoalHandle::SharedPtr goal_handle)
{
  client_goal_response_time_ = this->now();
  // Check if the goal was rejected by the server
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    // Reset the timer to try sending the goal again
    return;
  }

  // Store the goal handle for future reference
  goal_handle_ = goal_handle;

  // Reset the cancel timeout timer to start counting down
  cancel_timeout_timer_->reset();
}

void AnytimeActionClient::feedback_callback(
  AnytimeGoalHandle::SharedPtr goal_handle, const std::shared_ptr<const Anytime::Feedback> feedback)
{
  (void)goal_handle;
  // Log the feedback received from the action server
  RCLCPP_DEBUG(this->get_logger(), "Next number in the sequence: %f", feedback->feedback);
}

void AnytimeActionClient::result_callback(const AnytimeGoalHandle::WrappedResult & result)
{
  client_result_time_ = this->now();
  cancel_timeout_timer_->cancel();
  // Log the result based on the result code
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      // If the goal succeeded, log the result
      RCLCPP_DEBUG(this->get_logger(), "Result received");
      // print the number of iterations
      RCLCPP_DEBUG(this->get_logger(), "Number of iterations: %d", result.result->iterations);
      print_time_differences(result);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      // If the goal was aborted, log an error
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      // If the goal was canceled, log an error and the result after cancel
      RCLCPP_DEBUG(this->get_logger(), "Goal was canceled");
      // print the number of iterations
      RCLCPP_DEBUG(this->get_logger(), "Number of iterations: %d", result.result->iterations);
      print_time_differences(result);
      break;
    default:
      // If the result code is unknown, log an error
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
  }
}

void AnytimeActionClient::cancel_timeout_callback()
{
  RCLCPP_DEBUG(this->get_logger(), "Timeout reached, canceling goal");

  // Cancel the timeout timer to prevent multiple cancel requests
  cancel_timeout_timer_->cancel();

  client_send_cancel_start_time_ = this->now();
  // Send a cancel request for the current goal
  action_client_->async_cancel_goal(goal_handle_);
  client_send_cancel_end_time_ = this->now();

  RCLCPP_DEBUG(this->get_logger(), "Cancel request sent");
}

void AnytimeActionClient::print_time_differences(const AnytimeGoalHandle::WrappedResult & result)
{
  // Extract timestamps from client side
  RCLCPP_DEBUG(this->get_logger(), "Extracting raw timestamps");

  // Extract timestamps from server side (sent in the result)
  rclcpp::Time action_server_receive_time(
    result.result->action_server_receive.sec, result.result->action_server_receive.nanosec);
  rclcpp::Time action_server_accept_time(
    result.result->action_server_accept.sec, result.result->action_server_accept.nanosec);
  rclcpp::Time action_server_start_time(
    result.result->action_server_start.sec, result.result->action_server_start.nanosec);
  rclcpp::Time action_server_cancel_time(
    result.result->action_server_cancel.sec, result.result->action_server_cancel.nanosec);
  rclcpp::Time action_server_send_result_time(
    result.result->action_server_send_result.sec, result.result->action_server_send_result.nanosec);

  rclcpp::Time batch_time(result.result->batch_time.sec, result.result->batch_time.nanosec);

  // Log raw timestamps in nanoseconds
  RCLCPP_DEBUG(this->get_logger(), "Raw timestamp data (nanoseconds):");
  RCLCPP_DEBUG(
    this->get_logger(), "client_goal_start_time: %ld", client_goal_start_time_.nanoseconds());
  RCLCPP_DEBUG(
    this->get_logger(), "client_send_start_time: %ld", client_send_start_time_.nanoseconds());
  RCLCPP_DEBUG(
    this->get_logger(), "client_send_end_time: %ld", client_send_end_time_.nanoseconds());
  RCLCPP_DEBUG(
    this->get_logger(), "client_goal_response_time: %ld", client_goal_response_time_.nanoseconds());
  RCLCPP_DEBUG(
    this->get_logger(), "client_send_cancel_start_time: %ld",
    client_send_cancel_start_time_.nanoseconds());
  RCLCPP_DEBUG(
    this->get_logger(), "client_send_cancel_end_time: %ld",
    client_send_cancel_end_time_.nanoseconds());
  RCLCPP_DEBUG(this->get_logger(), "client_result_time: %ld", client_result_time_.nanoseconds());
  RCLCPP_DEBUG(
    this->get_logger(), "action_server_receive_time: %ld",
    action_server_receive_time.nanoseconds());
  RCLCPP_DEBUG(
    this->get_logger(), "action_server_accept_time: %ld", action_server_accept_time.nanoseconds());
  RCLCPP_DEBUG(
    this->get_logger(), "action_server_start_time: %ld", action_server_start_time.nanoseconds());
  RCLCPP_DEBUG(
    this->get_logger(), "action_server_cancel_time: %ld", action_server_cancel_time.nanoseconds());
  RCLCPP_DEBUG(
    this->get_logger(), "action_server_send_result_time: %ld",
    action_server_send_result_time.nanoseconds());
  RCLCPP_DEBUG(this->get_logger(), "batch_time_ns: %ld", batch_time.nanoseconds());
  RCLCPP_DEBUG(this->get_logger(), "iterations: %d", result.result->iterations);
  RCLCPP_DEBUG(this->get_logger(), "batch_size: %d", result.result->batch_size);

  // Create results directory if it doesn't exist
  std::string results_dir = "results";
  std::filesystem::create_directories(results_dir);

  // Use the provided filename instead of generating one
  std::string filename = results_dir + "/" + result_filename_ + ".csv";

  RCLCPP_DEBUG(this->get_logger(), "Using output file: %s", filename.c_str());

  bool file_exists = std::ifstream(filename).good();

  std::ofstream file;
  file.open(filename, std::ios::app);  // Append mode

  // Write header if file is new
  if (!file_exists) {
    file << "client_goal_start,client_send_start,client_send_end,client_goal_response,"
         << "client_send_cancel_start,client_send_cancel_end,client_result,"
         << "server_receive,server_accept,server_start,server_cancel,server_send_result,"
         << "batch_time_ns,iterations,batch_size\n";
  }

  // Write raw timestamp data
  file << client_goal_start_time_.nanoseconds() << "," << client_send_start_time_.nanoseconds()
       << "," << client_send_end_time_.nanoseconds() << ","
       << client_goal_response_time_.nanoseconds() << ","
       << client_send_cancel_start_time_.nanoseconds() << ","
       << client_send_cancel_end_time_.nanoseconds() << "," << client_result_time_.nanoseconds()
       << "," << action_server_receive_time.nanoseconds() << ","
       << action_server_accept_time.nanoseconds() << "," << action_server_start_time.nanoseconds()
       << "," << action_server_cancel_time.nanoseconds() << ","
       << action_server_send_result_time.nanoseconds() << "," << batch_time.nanoseconds() << ","
       << result.result->iterations << "," << result.result->batch_size << "\n";

  file.close();

  RCLCPP_DEBUG(this->get_logger(), "Raw timestamp data saved to %s", filename.c_str());
}