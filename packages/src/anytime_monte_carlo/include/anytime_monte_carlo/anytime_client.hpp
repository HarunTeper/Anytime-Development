#ifndef ANYTIME_CLIENT_HPP
#define ANYTIME_CLIENT_HPP

#include "anytime_interfaces/action/anytime.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// Class definition for the Anytime Action Client
class AnytimeActionClient : public rclcpp::Node {
 public:
  // Constructor
  AnytimeActionClient(const rclcpp::NodeOptions& options);

  // Destructor
  ~AnytimeActionClient();

  // Type aliases for convenience
  using Anytime = anytime_interfaces::action::Anytime;
  using AnytimeGoalHandle = rclcpp_action::ClientGoalHandle<Anytime>;

 private:
  // Action client for the Anytime action
  rclcpp_action::Client<anytime_interfaces::action::Anytime>::SharedPtr
      action_client_;

  // Timer for sending goals
  rclcpp::TimerBase::SharedPtr timer_ = nullptr;

  // Timer for cancel timeout
  rclcpp::TimerBase::SharedPtr cancel_timeout_timer_ = nullptr;

  // Handle for the current goal
  AnytimeGoalHandle::SharedPtr goal_handle_ = nullptr;

  // Function to send a goal to the action server
  void send_goal();

  // Callback for goal response
  void goal_response_callback(std::shared_ptr<AnytimeGoalHandle> goal_handle);

  // Callback for feedback from the action server
  void feedback_callback(
      AnytimeGoalHandle::SharedPtr,
      const std::shared_ptr<const Anytime::Feedback> feedback);

  // Callback for result from the action server
  void result_callback(const AnytimeGoalHandle::WrappedResult& result);

  // Callback for cancel timeout
  void cancel_timeout_callback();
};

// Register the node as a component
RCLCPP_COMPONENTS_REGISTER_NODE(AnytimeActionClient)

#endif  // ANYTIME_CLIENT_HPP
