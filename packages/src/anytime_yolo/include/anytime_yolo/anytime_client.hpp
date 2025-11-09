#ifndef ANYTIME_CLIENT_HPP
#define ANYTIME_CLIENT_HPP

#include "anytime_core/anytime_client_base.hpp"
#include "anytime_core/anytime_waitable.hpp"
#include "anytime_interfaces/action/yolo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

// Class definition for the Anytime Action Client
class AnytimeActionClient : public anytime_core::AnytimeClientBase<anytime_interfaces::action::Yolo>
{
public:
  // Type aliases for convenience
  using Anytime = anytime_interfaces::action::Yolo;
  using AnytimeGoalHandle = rclcpp_action::ClientGoalHandle<Anytime>;

  // Constructor
  AnytimeActionClient(const rclcpp::NodeOptions & options);

  // Destructor
  ~AnytimeActionClient();

private:
  // Subscription for camera images
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

  // Publisher for detection images
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detection_image_publisher_;

  // Publisher for detection results
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_publisher_;

  // AnytimeWaitable for cancel requests
  std::shared_ptr<anytime_core::AnytimeWaitable> cancel_waitable_;

  // Current image buffer
  sensor_msgs::msg::Image::SharedPtr current_image_;

  // Store the last goal message for retry on rejection
  Anytime::Goal last_goal_;

  // Flag to track if a cancel request is in progress
  bool is_cancelling_ = false;

  // Number of layers to process before canceling
  int cancel_after_layers_;

  // Boolean flag for cancel_layer_score
  bool cancel_layer_score_ = false;

  // Score threshold for cancel_layer_score mode
  double score_threshold_ = 0.7;

  // Target class ID for cancel_layer_score mode
  std::string target_class_id_ = "9";

  // Function to send a goal to the action server
  void send_goal(const Anytime::Goal & goal_msg);

  // Callback for cancel receive
  void cancel_response_callback(
    const std::shared_ptr<action_msgs::srv::CancelGoal_Response> & cancel_response);

  // Callback for cancel waitable
  void cancel_callback();

  // Callback for camera image subscription
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  // Domain-specific implementations from base class
  void post_processing(const AnytimeGoalHandle::WrappedResult & result) override;
  void log_result(const AnytimeGoalHandle::WrappedResult & result) override;
  void process_feedback(
    AnytimeGoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const Anytime::Feedback> feedback) override;
  void on_goal_rejected() override;
  void on_goal_accepted(AnytimeGoalHandle::SharedPtr goal_handle) override;
  void cleanup_after_result() override;
};

#endif  // ANYTIME_CLIENT_HPP
