#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "anytime_interfaces/action/anytime.hpp"

class AnytimeActionClient : public rclcpp::Node {
 public:
  AnytimeActionClient(const rclcpp::NodeOptions& options);
  ~AnytimeActionClient();

  using Anytime = anytime_interfaces::action::Anytime;
  using AnytimeGoalHandle = rclcpp_action::ClientGoalHandle<Anytime>;

 private:
  rclcpp_action::Client<anytime_interfaces::action::Anytime>::SharedPtr
      action_client_;

  void send_goal();
  void goal_response_callback(std::shared_ptr<AnytimeGoalHandle> goal_handle);
  void feedback_callback(
      AnytimeGoalHandle::SharedPtr,
      const std::shared_ptr<const Anytime::Feedback> feedback);
  void result_callback(const AnytimeGoalHandle::WrappedResult& result);
  void receive_goal_handle();
  void receive_result();
  void cancel_goal();
  void cancel_timeout_callback();

  // cancel response with cancel callback
  //  create a std::function<void (typename CancelResponse::SharedPtr)> for the
  //  cancel callback (with CancelResponse = typename
  //  ActionT::Impl::CancelGoalService::Response)
  void cancel_response_callback(
      std::shared_ptr<action_msgs::srv::CancelGoal::Response>);

  rclcpp::TimerBase::SharedPtr timer_ = nullptr;
  rclcpp::TimerBase::SharedPtr goal_handle_timer_ = nullptr;
  rclcpp::TimerBase::SharedPtr result_timer_ = nullptr;
  rclcpp::TimerBase::SharedPtr cancel_timer_ = nullptr;
  rclcpp::TimerBase::SharedPtr cancel_timeout_timer_ = nullptr;

  AnytimeGoalHandle::SharedPtr goal_handle_ = nullptr;

  std::shared_future<AnytimeGoalHandle::SharedPtr> goal_handle_future_;

  std::shared_future<AnytimeGoalHandle::WrappedResult> result_future_;

  AnytimeGoalHandle::WrappedResult wait_result_;

  std::shared_future<action_msgs::srv::CancelGoal_Response::SharedPtr>
      cancel_future_;

  rclcpp::Time start_time_;
  int timeout_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(AnytimeActionClient)