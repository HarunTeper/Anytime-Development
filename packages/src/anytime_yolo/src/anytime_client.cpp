#include "anytime_yolo/anytime_client.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#include <rclcpp/logging.hpp>

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
  // Initialize is_cancelling_ to false
  is_cancelling_ = false;

  // Declare domain-specific parameters with default values
  this->declare_parameter("image_topic", "video_frames");
  this->declare_parameter("cancel_after_layers", 10);
  this->declare_parameter("cancel_layer_score", false);
  this->declare_parameter("score_threshold", 0.7);
  this->declare_parameter("target_class_id", "9");

  std::string image_topic = this->get_parameter("image_topic").as_string();
  cancel_after_layers_ = this->get_parameter("cancel_after_layers").as_int();
  cancel_layer_score_ = this->get_parameter("cancel_layer_score").as_bool();
  score_threshold_ = this->get_parameter("score_threshold").as_double();
  target_class_id_ = this->get_parameter("target_class_id").as_string();

  RCLCPP_INFO(this->get_logger(), "YOLO Action Client initialized with parameters:");
  RCLCPP_INFO(this->get_logger(), "  image_topic: %s", image_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "  cancel_after_layers: %d", cancel_after_layers_);
  RCLCPP_INFO(
    this->get_logger(), "  cancel_layer_score: %s", cancel_layer_score_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  score_threshold: %.2f", score_threshold_);
  RCLCPP_INFO(this->get_logger(), "  target_class_id: %s", target_class_id_.c_str());

  RCLCPP_DEBUG(this->get_logger(), "Image topic: %s", image_topic.c_str());
  RCLCPP_DEBUG(this->get_logger(), "Cancel after layers: %d", cancel_after_layers_);

  // Initialize the cancel waitable
  cancel_waitable_ =
    std::make_shared<anytime_core::AnytimeWaitable>([this]() { this->cancel_callback(); });

  // Add the waitable to the node's waitables interface
  this->get_node_waitables_interface()->add_waitable(
    cancel_waitable_, this->get_node_base_interface()->get_default_callback_group());

  // Initialize the image subscription
  image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic, 10,
    [this](const sensor_msgs::msg::Image::SharedPtr msg) { this->image_callback(msg); });

  // Initialize the detection image publisher
  detection_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("images", 10);

  detection_publisher_ =
    this->create_publisher<vision_msgs::msg::Detection2DArray>("/detections", 10);
}

AnytimeActionClient::~AnytimeActionClient() {}

// Fill the image callback implementation with image data
void AnytimeActionClient::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  Anytime::Goal goal_msg;

  // Fill in the goal message with data from the image message
  goal_msg.image = *msg;
  goal_msg.image.header.stamp = msg->header.stamp;

  if (!current_image_) {
    current_image_ = std::make_shared<sensor_msgs::msg::Image>(*msg);
    current_image_->header.stamp = msg->header.stamp;
  } else {
    return;
  }

  send_goal(goal_msg);
}

void AnytimeActionClient::send_goal(const Anytime::Goal & goal_msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Sending goal info");

  // Store the goal for potential retry
  last_goal_ = goal_msg;

  // Use the base class helper to send the goal
  send_goal_to_server(goal_msg);
}

void AnytimeActionClient::on_goal_rejected()
{
  RCLCPP_WARN(this->get_logger(), "Goal rejected by server, retrying...");

  // Resend the goal
  send_goal_to_server(last_goal_);
}

void AnytimeActionClient::on_goal_accepted(AnytimeGoalHandle::SharedPtr goal_handle)
{
  RCLCPP_DEBUG(
    this->get_logger(), "[Goal ID: %s] Goal accepted by server",
    rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
}

void AnytimeActionClient::process_feedback(
  AnytimeGoalHandle::SharedPtr goal_handle, const std::shared_ptr<const Anytime::Feedback> feedback)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Feedback received for unknown goal handle");
    return;
  }

  std::string goal_id_str = rclcpp_action::to_string(goal_handle->get_goal_id());
  // Log the feedback message
  RCLCPP_DEBUG(this->get_logger(), "[Goal ID: %s] Feedback received", goal_id_str.c_str());
  RCLCPP_DEBUG(
    this->get_logger(), "[Goal ID: %s] Processed layers: %d", goal_id_str.c_str(),
    feedback->processed_layers);

  // Print information about detections with highest score
  if (!feedback->detections.empty()) {
    // print the number of detections
    RCLCPP_DEBUG(
      this->get_logger(), "[Goal ID: %s] Number of detections: %zu", goal_id_str.c_str(),
      feedback->detections.size());

    // Find detection with the highest score
    const auto & detections = feedback->detections;

    // For each detection, find the result with the highest score
    for (size_t i = 0; i < detections.size(); ++i) {
      const auto & detection = detections[i];

      if (detection.results.empty()) {
        continue;
      }

      // Find the result with the highest score in this detection
      auto highest_score_result = std::max_element(
        detection.results.begin(), detection.results.end(),
        [](const auto & a, const auto & b) { return a.hypothesis.score < b.hypothesis.score; });

      // Print the highest score and its class ID
      RCLCPP_DEBUG(
        this->get_logger(), "[Goal ID: %s] Detection %zu: Class ID = %s, Score = %.3f",
        goal_id_str.c_str(), i, highest_score_result->hypothesis.class_id.c_str(),
        highest_score_result->hypothesis.score);
    }
  } else {
    RCLCPP_DEBUG(
      this->get_logger(), "[Goal ID: %s] No detections in feedback", goal_id_str.c_str());
  }

  if (cancel_layer_score_) {
    // Cancel if high score for target class is detected
    if (!feedback->detections.empty()) {
      const auto & detections = feedback->detections;
      for (size_t i = 0; i < detections.size(); ++i) {
        const auto & detection = detections[i];
        if (detection.results.empty()) {
          continue;
        }
        // Check if any result in this detection has score >= threshold and class_id matches target
        auto found = std::find_if(
          detection.results.begin(), detection.results.end(), [this](const auto & res) {
            return res.hypothesis.score >= score_threshold_ &&
                   res.hypothesis.class_id == target_class_id_;
          });
        if (found != detection.results.end() && !is_cancelling_) {
          RCLCPP_INFO(
            this->get_logger(),
            "Canceling goal due to high score (%.2f) for class %s after %d layers",
            found->hypothesis.score, target_class_id_.c_str(), feedback->processed_layers);
          cancel_waitable_->trigger();
          return;
        }
      }
    }
  } else {
    // Cancel after cancel_after_layers
    if (feedback->processed_layers >= cancel_after_layers_ && !is_cancelling_) {
      RCLCPP_INFO(
        this->get_logger(), "Notifying cancel waitable after %d layers",
        feedback->processed_layers);
      cancel_waitable_->trigger();
    }
  }
}

void AnytimeActionClient::log_result(const AnytimeGoalHandle::WrappedResult & result)
{
  (void)result;
  RCLCPP_DEBUG(
    this->get_logger(), "[Goal ID: %s] Result received",
    rclcpp_action::to_string(goal_handle_->get_goal_id()).c_str());
}

void AnytimeActionClient::post_processing(const AnytimeGoalHandle::WrappedResult & result)
{
  std::string goal_id_str = rclcpp_action::to_string(goal_handle_->get_goal_id());
  RCLCPP_DEBUG(this->get_logger(), "[Goal ID: %s] Publishing detection image", goal_id_str.c_str());
  detection_image_publisher_->publish(*current_image_);

  // Create a Detection2DArray message for publishing
  vision_msgs::msg::Detection2DArray detection_array;
  detection_array.header.stamp = current_image_->header.stamp;

  // Copy detections from the result
  detection_array.detections = result.result->detections;

  // Log number of detections
  RCLCPP_DEBUG(
    this->get_logger(), "[Goal ID: %s] Publishing %ld detections", goal_id_str.c_str(),
    result.result->detections.size());

  // Publish detections
  detection_publisher_->publish(detection_array);
}

void AnytimeActionClient::cleanup_after_result()
{
  current_image_.reset();
  // Reset the cancellation flag when we receive a result
  is_cancelling_ = false;
}

void AnytimeActionClient::cancel_callback()
{
  if (is_cancelling_ || !goal_handle_) {
    return;
  }

  std::string goal_id_str = rclcpp_action::to_string(goal_handle_->get_goal_id());
  RCLCPP_DEBUG(
    this->get_logger(), "[Goal ID: %s] Cancel waitable triggered, cancelling goal",
    goal_id_str.c_str());
  // Set the cancellation flag to true to prevent multiple cancellations
  is_cancelling_ = true;

  // Record cancel sent timestamp
  auto cancel_sent_time = this->now().nanoseconds();
  TRACE_ANYTIME_CLIENT_CANCEL_SENT(this, cancel_sent_time);

  // Send a cancel request for the current goal
  action_client_->async_cancel_goal(
    goal_handle_, [this](std::shared_ptr<action_msgs::srv::CancelGoal_Response> cancel_response) {
      this->cancel_response_callback(cancel_response);
    });

  RCLCPP_DEBUG(this->get_logger(), "[Goal ID: %s] Cancel request sent", goal_id_str.c_str());
}

void AnytimeActionClient::cancel_response_callback(
  const std::shared_ptr<action_msgs::srv::CancelGoal_Response> & cancel_response)
{
  (void)cancel_response;
  if (goal_handle_) {
    RCLCPP_DEBUG(
      this->get_logger(), "[Goal ID: %s] Cancel request accepted by server",
      rclcpp_action::to_string(goal_handle_->get_goal_id()).c_str());
  }
}

// Register the component
RCLCPP_COMPONENTS_REGISTER_NODE(AnytimeActionClient)