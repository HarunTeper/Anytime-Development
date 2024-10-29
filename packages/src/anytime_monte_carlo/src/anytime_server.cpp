#include "anytime_monte_carlo/anytime_server.hpp"

// Constructor for the AnytimeActionServer class
AnytimeActionServer::AnytimeActionServer(const rclcpp::NodeOptions& options)
    : Node("anytime_action_server", options) {
  RCLCPP_INFO(this->get_logger(), "Starting Anytime action server");

  // Create the action server
  action_server_ =
      rclcpp_action::create_server<anytime_interfaces::action::Anytime>(
          this, "anytime",
          // Lambda function to handle goal requests
          [this](
              const rclcpp_action::GoalUUID uuid,
              std::shared_ptr<const anytime_interfaces::action::Anytime::Goal>
                  goal) { return this->handle_goal(uuid, goal); },
          // Lambda function to handle cancel requests
          [this](const std::shared_ptr<AnytimeGoalHandle> goal_handle) {
            return this->handle_cancel(goal_handle);
          },
          // Lambda function to handle accepted goals
          [this](const std::shared_ptr<AnytimeGoalHandle> goal_handle) {
            return this->handle_accepted(goal_handle);
          });

  // read the ros2 paramter anytime_active and anytime_blocking
  bool anytime_active = this->declare_parameter("anytime_active", false);
  bool anytime_blocking = this->declare_parameter("anytime_blocking", false);

  RCLCPP_INFO(this->get_logger(), "anytime_active: %d", anytime_active);
  RCLCPP_INFO(this->get_logger(), "anytime_blocking: %d", anytime_blocking);

  anytime_waitable_ = std::make_shared<AnytimeWaitable>([this]() {});

  // Create a shared pointer to a MonteCarloPi object with the values of the
  // parameters as the template arguments
  if (anytime_active && anytime_blocking) {
    auto monte_carlo_pi =
        std::make_shared<MonteCarloPi<true, true>>(anytime_waitable_);
  } else if (anytime_active && !anytime_blocking) {
    auto monte_carlo_pi =
        std::make_shared<MonteCarloPi<true, false>>(anytime_waitable_);
  } else if (!anytime_active && anytime_blocking) {
    auto monte_carlo_pi =
        std::make_shared<MonteCarloPi<false, true>>(anytime_waitable_);
  } else {
    auto monte_carlo_pi =
        std::make_shared<MonteCarloPi<false, false>>(anytime_waitable_);
  }
}

// Destructor for the AnytimeActionServer class
AnytimeActionServer::~AnytimeActionServer() {}

// Handle goal request
rclcpp_action::GoalResponse AnytimeActionServer::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    const std::shared_ptr<const anytime_interfaces::action::Anytime::Goal>
        goal) {
  RCLCPP_INFO(this->get_logger(), "Received goal request with number %d",
              goal->goal);
  (void)uuid;  // Suppress unused variable warning
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// Handle cancel request
rclcpp_action::CancelResponse AnytimeActionServer::handle_cancel(
    const std::shared_ptr<AnytimeGoalHandle> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;  // Suppress unused variable warning
  return rclcpp_action::CancelResponse::ACCEPT;
}

void AnytimeActionServer::execute(
    const std::shared_ptr<AnytimeGoalHandle> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  auto feedback =
      std::make_shared<anytime_interfaces::action::Anytime::Feedback>();
  auto& feedback_value = feedback->feedback;
  feedback_value = 0;
  auto result = std::make_shared<anytime_interfaces::action::Anytime::Result>();
  result->result = 0;

  int count_total = 0;
  int count_inside = 0;
  int count_outside = 0;

  float_t x = 0.0;
  float_t y = 0.0;

  // start time
  auto start = rclcpp::Clock().now();

  for (int i = 1; i <= goal_handle->get_goal()->goal; i++) {
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal was canceled");
      result->result = feedback_value;
      goal_handle->canceled(result);
      return;
    }
    // sample x and y between 0 and 1, and if the length of the vector is
    // greater than one, add count to count_outside, otherwise add to
    // count_inside
    x = (float_t)rand() / RAND_MAX;
    y = (float_t)rand() / RAND_MAX;

    if (sqrt(pow(x, 2) + pow(y, 2)) <= 1) {
      count_inside++;
    } else {
      count_outside++;
    }
    count_total++;

    feedback_value = 4 * (float_t)count_inside / count_total;

    // goal_handle->publish_feedback(feedback);
  }

  result->result = feedback_value;

  // end time
  auto end = rclcpp::Clock().now();
  // calculate the duration
  auto duration = end - start;
  // print the duration in msec and sec
  RCLCPP_INFO(this->get_logger(), "Duration: %f msec",
              duration.nanoseconds() / 1e6);
  goal_handle->succeed(result);

  RCLCPP_INFO(this->get_logger(), "Goal was completed");
}

void AnytimeActionServer::handle_accepted(
    const std::shared_ptr<AnytimeGoalHandle> goal_handle) {
  // Create a new thread to execute the goal
  std::thread([this, goal_handle]() {
    // Execute the goal
    this->execute(goal_handle);
  }).detach();  // Detach the thread to allow it to run independently
}