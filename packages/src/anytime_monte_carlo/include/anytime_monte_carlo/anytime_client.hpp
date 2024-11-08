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

  // Function to print time differences between action result timestamps
  void print_time_differences(const AnytimeGoalHandle::WrappedResult& result);

  // receive time
  rclcpp::Time receive_time_ = rclcpp::Time(0, 0);

  // cancel send time
  rclcpp::Time cancel_send_time_ = rclcpp::Time(0, 0);

  // Data structures to store intervals
  std::vector<int64_t> intervals1_;
  std::vector<int64_t> intervals2_;
  std::vector<int64_t> intervals3_;
  std::vector<int64_t> intervals4_;
  std::vector<int64_t> intervals5_;
  std::vector<int64_t> intervals6_;

  std::vector<int64_t> iterations_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(AnytimeActionClient)

#endif  // ANYTIME_CLIENT_HPP
