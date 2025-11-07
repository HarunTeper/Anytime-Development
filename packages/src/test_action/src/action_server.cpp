#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "test_action/action/test_action.hpp"
#include "test_action/test_waitable.hpp"

#include <chrono>
#include <memory>

class TestActionServer : public rclcpp::Node
{
public:
  using TestAction = test_action::action::TestAction;
  using GoalHandleTestAction = rclcpp_action::ServerGoalHandle<TestAction>;

  explicit TestActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("test_action_server", options),
    current_goal_handle_(nullptr),
    waitable_active_(false),
    waitable_iteration_count_(0)
  {
    using namespace std::placeholders;

    RCLCPP_INFO(this->get_logger(), "Starting Test Action Server");

    this->action_server_ = rclcpp_action::create_server<TestAction>(
      this, "test_action", std::bind(&TestActionServer::handle_goal, this, _1, _2),
      std::bind(&TestActionServer::handle_cancel, this, _1),
      std::bind(&TestActionServer::handle_accepted, this, _1));

    // Create a separate callback group for the waitable
    waitable_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Create the test waitable with execution callback
    test_waitable_ = std::make_shared<test_action::TestWaitable>(
      std::bind(&TestActionServer::waitable_callback, this));

    // Add the waitable to the node with its own callback group
    this->get_node_waitables_interface()->add_waitable(test_waitable_, waitable_callback_group_);

    RCLCPP_INFO(this->get_logger(), "Test Action Server ready");
  }

private:
  rclcpp_action::Server<TestAction>::SharedPtr action_server_;
  std::shared_ptr<GoalHandleTestAction> current_goal_handle_;
  std::shared_ptr<test_action::TestWaitable> test_waitable_;
  rclcpp::CallbackGroup::SharedPtr waitable_callback_group_;
  bool waitable_active_;
  int waitable_iteration_count_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const TestAction::Goal> goal)
  {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received goal request with goal_id: %d", goal->goal_id);

    // Reject if we're already processing a goal
    if (current_goal_handle_ != nullptr) {
      RCLCPP_WARN(this->get_logger(), "Already processing a goal, rejecting new goal");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTestAction> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleTestAction> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Goal accepted, starting execution via waitable");

    // Store the goal handle
    current_goal_handle_ = goal_handle;

    // Trigger the waitable to start its execution
    waitable_active_ = true;
    waitable_iteration_count_ = 0;
    test_waitable_->trigger();
    RCLCPP_INFO(this->get_logger(), "Waitable triggered");
  }

  void waitable_callback()
  {
    if (!waitable_active_) {
      RCLCPP_INFO(this->get_logger(), "Waitable callback called but not active, ignoring");
      return;
    }

    if (!current_goal_handle_) {
      RCLCPP_ERROR(this->get_logger(), "Waitable callback but no active goal handle!");
      waitable_active_ = false;
      return;
    }

    // Check if goal is being canceled
    if (current_goal_handle_->is_canceling()) {
      auto result = std::make_shared<TestAction::Result>();
      result->result_value = -1;
      result->message = "Goal was canceled";
      current_goal_handle_->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");

      // Clean up
      waitable_active_ = false;
      waitable_iteration_count_ = 0;
      current_goal_handle_ = nullptr;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Waitable callback - iteration %d", waitable_iteration_count_);

    // Perform placebo computation
    placebo_compute();

    // Publish feedback
    auto feedback = std::make_shared<TestAction::Feedback>();
    feedback->progress = (waitable_iteration_count_ + 1) * 20;
    current_goal_handle_->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "Publishing feedback: %d%% complete", feedback->progress);

    waitable_iteration_count_++;

    // Check if we're done (5 iterations)
    if (waitable_iteration_count_ >= 5) {
      const auto goal = current_goal_handle_->get_goal();
      auto result = std::make_shared<TestAction::Result>();
      result->result_value = goal->goal_id * 100;
      result->message = "Goal completed successfully";
      current_goal_handle_->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded with result: %d", result->result_value);

      // Clean up
      waitable_active_ = false;
      waitable_iteration_count_ = 0;
      current_goal_handle_ = nullptr;

      RCLCPP_INFO(this->get_logger(), "Waitable completed, ready for next goal");
    } else {
      // Trigger again for the next iteration
      test_waitable_->trigger();
      RCLCPP_INFO(this->get_logger(), "Waitable re-triggered for next iteration");
    }
  }

  void placebo_compute()
  {
    // Placebo computation - just waste some CPU cycles
    volatile double result = 0.0;
    for (int i = 0; i < 100000; ++i) {
      result += i * 0.001;
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TestActionServer>();

  // Use multi-threaded executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  RCLCPP_INFO(node->get_logger(), "Multi-threaded executor spinning...");
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
