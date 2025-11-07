#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "test_action/action/test_action.hpp"

#include <chrono>
#include <memory>

class TestActionClient : public rclcpp::Node
{
public:
  using TestAction = test_action::action::TestAction;
  using GoalHandleTestAction = rclcpp_action::ClientGoalHandle<TestAction>;

  explicit TestActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("test_action_client", options), goal_counter_(0)
  {
    RCLCPP_INFO(this->get_logger(), "Starting Test Action Client");

    this->client_ptr_ = rclcpp_action::create_client<TestAction>(this, "test_action");

    // Timer to send goals every second
    this->timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&TestActionClient::send_goal, this));

    RCLCPP_INFO(this->get_logger(), "Test Action Client ready");
  }

private:
  rclcpp_action::Client<TestAction>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  int goal_counter_;

  void send_goal()
  {
    using namespace std::placeholders;

    // Check if server is available
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Action server not available, waiting...");
      return;
    }

    goal_counter_++;
    auto goal_msg = TestAction::Goal();
    goal_msg.goal_id = goal_counter_;

    RCLCPP_INFO(this->get_logger(), "Sending goal #%d", goal_counter_);

    auto send_goal_options = rclcpp_action::Client<TestAction>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      std::bind(&TestActionClient::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
      std::bind(&TestActionClient::feedback_callback, this, _1, _2);

    send_goal_options.result_callback = std::bind(&TestActionClient::result_callback, this, _1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(const GoalHandleTestAction::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleTestAction::SharedPtr, const std::shared_ptr<const TestAction::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Received feedback: %d%% complete", feedback->progress);
  }

  void result_callback(const GoalHandleTestAction::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(
          this->get_logger(), "Goal succeeded! Result: %d, Message: %s",
          result.result->result_value, result.result->message.c_str());
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TestActionClient>();

  // Use multi-threaded executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  RCLCPP_INFO(node->get_logger(), "Multi-threaded executor spinning...");
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
