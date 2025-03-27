#ifndef ANYTIME_CLIENT_HPP
#define ANYTIME_CLIENT_HPP

#include "anytime_interfaces/action/yolo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/image.hpp"               // Added include for image messages
#include "vision_msgs/msg/detection2_d_array.hpp"  // Added include for detection messages

// Class definition for the Anytime Action Client
class AnytimeActionClient : public rclcpp::Node
{
public:
  // Constructor
  AnytimeActionClient(const rclcpp::NodeOptions & options);

  // Destructor
  ~AnytimeActionClient();

  // Type aliases for convenience
  using Anytime = anytime_interfaces::action::Yolo;
  using AnytimeGoalHandle = rclcpp_action::ClientGoalHandle<Anytime>;

private:
  // Action client for the Anytime action
  rclcpp_action::Client<Anytime>::SharedPtr action_client_;

  // Subscription for camera images
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

  // Publisher for detection images
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detection_image_publisher_;

  // Publisher for detection results
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_publisher_;

  // Current image buffer
  sensor_msgs::msg::Image::SharedPtr current_image_;

  // Timer for sending goals
  rclcpp::TimerBase::SharedPtr timer_ = nullptr;

  // Timer for cancel timeout
  rclcpp::TimerBase::SharedPtr cancel_timeout_timer_ = nullptr;

  // Handle for the current goal
  AnytimeGoalHandle::SharedPtr goal_handle_ = nullptr;

  // Filename for storing results
  std::string result_filename_;

  // Function to send a goal to the action server
  void send_goal(const Anytime::Goal & goal_msg);

  // Callback for goal response
  void goal_response_callback(std::shared_ptr<AnytimeGoalHandle> goal_handle);

  // Callback for feedback from the action server
  void feedback_callback(
    AnytimeGoalHandle::SharedPtr, const std::shared_ptr<const Anytime::Feedback> feedback);

  // Callback for result from the action server
  void result_callback(const AnytimeGoalHandle::WrappedResult & result);

  // Callback for cancel timeout
  void cancel_timeout_callback();

  // Function to print time differences between action result timestamps
  void print_time_differences(const AnytimeGoalHandle::WrappedResult & result);

  // Callback for camera image subscription
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  // Function to handle post-processing of the result
  void post_processing(const AnytimeGoalHandle::WrappedResult & result);

  // Variables to store timestamps
  // Timestamps for client-side events
  rclcpp::Time client_goal_start_time_;         // When the goal was created
  rclcpp::Time client_send_start_time_;         // Right before sending the goal
  rclcpp::Time client_send_end_time_;           // Right after sending the goal
  rclcpp::Time client_goal_response_time_;      // When goal response was received
  rclcpp::Time client_result_time_;             // When result was received
  rclcpp::Time client_send_cancel_start_time_;  // Before sending cancel request
  rclcpp::Time client_send_cancel_end_time_;    // After sending cancel request

  // Data structures to store timing intervals for statistics
  std::vector<int64_t> intervals_;        // Store time intervals in nanoseconds
  std::vector<int32_t> iterations_data_;  // Store iteration counts

  // Legacy data structures (kept for compatibility)
  std::vector<int64_t> intervals1_;
  std::vector<int64_t> intervals2_;
  std::vector<int64_t> intervals3_;
  std::vector<int64_t> intervals4_;
  std::vector<int64_t> intervals5_;
  std::vector<int64_t> intervals6_;
  std::vector<int64_t> iterations_;
};

#endif  // ANYTIME_CLIENT_HPP
