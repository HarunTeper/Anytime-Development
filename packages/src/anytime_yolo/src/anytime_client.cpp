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
  RCLCPP_INFO(this->get_logger(), "Starting Anytime action client");
  // Initialize is_cancelling_ to false
  is_cancelling_ = false;

  // Declare parameters with default values
  this->declare_parameter("result_filename", "anytime_results");
  this->declare_parameter("image_topic", "video_frames");
  this->declare_parameter("cancel_after_layers", 10);

  result_filename_ = this->get_parameter("result_filename").as_string();
  std::string image_topic = this->get_parameter("image_topic").as_string();
  cancel_after_layers_ = this->get_parameter("cancel_after_layers").as_int();

  RCLCPP_INFO(this->get_logger(), "Result filename: %s", result_filename_.c_str());
  RCLCPP_INFO(this->get_logger(), "Image topic: %s", image_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Cancel after layers: %d", cancel_after_layers_);

  // Initialize the cancel waitable
  cancel_waitable_ = std::make_shared<AnytimeWaitable>([this]() { this->cancel_callback(); });

  // Add the waitable to the node's waitables interface
  this->get_node_waitables_interface()->add_waitable(
    cancel_waitable_, this->get_node_base_interface()->get_default_callback_group());

  // Initialize the image subscription
  image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic, 10,
    [this](const sensor_msgs::msg::Image::SharedPtr msg) { this->image_callback(msg); });

  // Initialize the detection image publisher
  detection_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("images", 10);

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
  goal_msg.image = *msg;
  goal_msg.image.header.stamp = msg->header.stamp;

  if (!current_image_) {
    current_image_ = std::make_shared<sensor_msgs::msg::Image>(*msg);
    current_image_->header.stamp = msg->header.stamp;
  }

  send_goal(goal_msg);
}

void AnytimeActionClient::send_goal(const Anytime::Goal & goal_msg)
{
  RCLCPP_INFO(this->get_logger(), "Sending goal info");

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
    return;
  }

  // Store the goal handle for future reference
  goal_handle_ = goal_handle;

  RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
}

void AnytimeActionClient::feedback_callback(
  AnytimeGoalHandle::SharedPtr goal_handle, const std::shared_ptr<const Anytime::Feedback> feedback)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Feedback received for unknown goal handle");
    return;
  }
  // Log the feedback message
  RCLCPP_INFO(this->get_logger(), "Feedback received");
  RCLCPP_INFO(this->get_logger(), "Processed layers: %d", feedback->processed_layers);
  if (feedback->processed_layers >= cancel_after_layers_ && !is_cancelling_) {
    RCLCPP_INFO(this->get_logger(), "Notifying cancel waitable");
    cancel_waitable_->notify();
  }
}

void AnytimeActionClient::cancel_callback()
{
  if (is_cancelling_ || !goal_handle_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Cancel waitable triggered, cancelling goal");
  // Set the cancellation flag to true to prevent multiple cancellations
  is_cancelling_ = true;

  client_send_cancel_start_time_ = this->now();

  // Send a cancel request for the current goal
  action_client_->async_cancel_goal(
    goal_handle_, [this](std::shared_ptr<action_msgs::srv::CancelGoal_Response> cancel_response) {
      this->cancel_response_callback(cancel_response);
    });

  client_send_cancel_end_time_ = this->now();
  RCLCPP_INFO(this->get_logger(), "Cancel request sent");
}

void AnytimeActionClient::cancel_response_callback(
  const std::shared_ptr<action_msgs::srv::CancelGoal_Response> & cancel_response)
{
  (void)cancel_response;
  RCLCPP_INFO(this->get_logger(), "Cancel request accepted by server");
}

void AnytimeActionClient::result_callback(const AnytimeGoalHandle::WrappedResult & result)
{
  client_result_time_ = this->now();

  // Log the result based on the result code
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      // If the goal succeeded, log the result
      RCLCPP_INFO(this->get_logger(), "Result received");
      post_processing(result);
      print_time_differences(result);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      // If the goal was aborted, log an error
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      // If the goal was canceled, log an error and the result after cancel
      RCLCPP_INFO(this->get_logger(), "Goal was canceled");
      post_processing(result);
      print_time_differences(result);
      break;
    default:
      // If the result code is unknown, log an error
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
  }
  current_image_.reset();
  // Reset the cancellation flag when we receive a result
  is_cancelling_ = false;
}

void AnytimeActionClient::post_processing(const AnytimeGoalHandle::WrappedResult & result)
{
  // print time between start and receive in milliseconds
  RCLCPP_INFO(
    this->get_logger(), "Time between start and receive: %ld ms",
    (client_result_time_ - client_goal_start_time_).nanoseconds() / 1000000);
  RCLCPP_INFO(this->get_logger(), "Publishing detection image");
  detection_image_publisher_->publish(*current_image_);

  // Create a Detection2DArray message for publishing
  vision_msgs::msg::Detection2DArray detection_array;
  detection_array.header.stamp = current_image_->header.stamp;

  // Copy detections from the result
  detection_array.detections = result.result->detections;

  // Log number of detections
  RCLCPP_INFO(this->get_logger(), "Publishing %ld detections", detection_array.detections.size());

  // Create publisher on first call if it doesn't exist
  if (!detection_publisher_) {
    detection_publisher_ =
      this->create_publisher<vision_msgs::msg::Detection2DArray>("/detections", 10);
  }

  // Publish detections
  detection_publisher_->publish(detection_array);
}

void AnytimeActionClient::print_time_differences(const AnytimeGoalHandle::WrappedResult & result)
{
  // Extract timestamps from client side
  RCLCPP_INFO(this->get_logger(), "Extracting raw timestamps");

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

  rclcpp::Time batch_time(
    result.result->average_batch_time.sec, result.result->average_batch_time.nanosec);

  int processed_layers = result.result->processed_layers;
  int result_processed_layers = result.result->result_processed_layers;

  // Log raw timestamps in nanoseconds
  RCLCPP_INFO(this->get_logger(), "Raw timestamp data (nanoseconds):");
  RCLCPP_INFO(
    this->get_logger(), "client_goal_start_time: %ld", client_goal_start_time_.nanoseconds());
  RCLCPP_INFO(
    this->get_logger(), "client_send_start_time: %ld", client_send_start_time_.nanoseconds());
  RCLCPP_INFO(this->get_logger(), "client_send_end_time: %ld", client_send_end_time_.nanoseconds());
  RCLCPP_INFO(
    this->get_logger(), "client_goal_response_time: %ld", client_goal_response_time_.nanoseconds());
  RCLCPP_INFO(
    this->get_logger(), "client_send_cancel_start_time: %ld",
    client_send_cancel_start_time_.nanoseconds());
  RCLCPP_INFO(
    this->get_logger(), "client_send_cancel_end_time: %ld",
    client_send_cancel_end_time_.nanoseconds());
  RCLCPP_INFO(this->get_logger(), "client_result_time: %ld", client_result_time_.nanoseconds());
  RCLCPP_INFO(
    this->get_logger(), "action_server_receive_time: %ld",
    action_server_receive_time.nanoseconds());
  RCLCPP_INFO(
    this->get_logger(), "action_server_accept_time: %ld", action_server_accept_time.nanoseconds());
  RCLCPP_INFO(
    this->get_logger(), "action_server_start_time: %ld", action_server_start_time.nanoseconds());
  RCLCPP_INFO(
    this->get_logger(), "action_server_cancel_time: %ld", action_server_cancel_time.nanoseconds());
  RCLCPP_INFO(
    this->get_logger(), "action_server_send_result_time: %ld",
    action_server_send_result_time.nanoseconds());
  RCLCPP_INFO(this->get_logger(), "batch_time_ns: %ld", batch_time.nanoseconds());
  RCLCPP_INFO(this->get_logger(), "processed_layers: %d", processed_layers);
  RCLCPP_INFO(this->get_logger(), "result_processed_layers: %d", result_processed_layers);

  // Create results directory if it doesn't exist
  std::string results_dir = "results";
  std::filesystem::create_directories(results_dir);

  // Use the provided filename instead of generating one
  std::string filename = results_dir + "/" + result_filename_ + ".csv";

  RCLCPP_INFO(this->get_logger(), "Using output file: %s", filename.c_str());

  bool file_exists = std::ifstream(filename).good();

  std::ofstream file;
  file.open(filename, std::ios::app);  // Append mode

  // Write header if file is new
  if (!file_exists) {
    file << "client_goal_start,client_send_start,client_send_end,client_goal_response,"
         << "client_send_cancel_start,client_send_cancel_end,client_result,"
         << "server_receive,server_accept,server_start,server_cancel,server_send_result,"
         << "batch_time_ns,processed_layers,result_processed_layers\n";
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
       << processed_layers << "," << result_processed_layers << "\n";

  file.close();

  RCLCPP_INFO(this->get_logger(), "Raw timestamp data saved to %s", filename.c_str());
}