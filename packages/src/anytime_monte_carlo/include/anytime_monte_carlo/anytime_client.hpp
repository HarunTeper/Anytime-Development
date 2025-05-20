#ifndef ANYTIME_CLIENT_HPP
#define ANYTIME_CLIENT_HPP

#include "anytime_interfaces/action/monte_carlo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Class definition for the Anytime Action Client
class AnytimeActionClient : public rclcpp::Node
{
public:
  // Constructor
  AnytimeActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Destructor
  ~AnytimeActionClient();

  // Type aliases for convenience
  using Anytime = anytime_interfaces::action::MonteCarlo;
  using AnytimeGoalHandle = rclcpp_action::ClientGoalHandle<Anytime>;

private:
  // Action client for the Anytime action
  rclcpp_action::Client<Anytime>::SharedPtr action_client_;

  // Timer for sending goals
  rclcpp::TimerBase::SharedPtr timer_ = nullptr;

  // Timer for cancel timeout
  rclcpp::TimerBase::SharedPtr cancel_timeout_timer_ = nullptr;

  // Handle for the current goal
  AnytimeGoalHandle::SharedPtr goal_handle_ = nullptr;

  // Filename for storing results
  std::string result_filename_;

  // Function to send a goal to the action server
  void send_goal();

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

  // Variables to store timestamps
  // Timestamps for client-side events
  rclcpp::Time client_goal_start_time_;         // When the goal was created
  rclcpp::Time client_send_start_time_;         // Right before sending the goal
  rclcpp::Time client_send_end_time_;           // Right after sending the goal
  rclcpp::Time client_goal_response_time_;      // When goal response was received
  rclcpp::Time client_result_time_;             // When result was received
  rclcpp::Time client_send_cancel_start_time_;  // Before sending cancel request
  rclcpp::Time client_send_cancel_end_time_;    // After sending cancel request
};

#endif  // ANYTIME_CLIENT_HPP
