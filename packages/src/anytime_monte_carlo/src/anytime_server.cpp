#include "anytime_monte_carlo/anytime_server.hpp"
#include <rclcpp/logging.hpp>

// Constructor for the AnytimeActionServer class
AnytimeActionServer::AnytimeActionServer(const rclcpp::NodeOptions& options)
    : Node("anytime_action_server", options) {
  RCLCPP_INFO(this->get_logger(), "Starting Anytime action server");

  // Create the action server
  // Create a callback group for the action server
  callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

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
          },
          rcl_action_server_get_default_options(), callback_group_);

  // read the ros2 paramter anytime_active and anytime_blocking
  bool anytime_active = this->declare_parameter("anytime_active", false);
  bool separate_thread = this->declare_parameter("separate_thread", false);

  RCLCPP_INFO(this->get_logger(), "anytime_active: %d", anytime_active);
  RCLCPP_INFO(this->get_logger(), "separate_thread: %d", separate_thread);

  // Create a shared pointer to a MonteCarloPi object with the node reference
  if (anytime_active && separate_thread) {
    RCLCPP_INFO(this->get_logger(),
                "Creating MonteCarloPi in active mode with separate thread");
    monte_carlo_pi_ = std::make_shared<MonteCarloPi<true, true>>(this);
  } else if (anytime_active && !separate_thread) {
    RCLCPP_INFO(this->get_logger(),
                "Creating MonteCarloPi in active mode without separate thread");
    monte_carlo_pi_ = std::make_shared<MonteCarloPi<true, false>>(this);
  } else if (!anytime_active && separate_thread) {
    RCLCPP_INFO(this->get_logger(),
                "Creating MonteCarloPi in inactive mode with separate thread");
    monte_carlo_pi_ = std::make_shared<MonteCarloPi<false, true>>(this);
  } else {
    RCLCPP_INFO(
        this->get_logger(),
        "Creating MonteCarloPi in inactive mode without separate thread");
    monte_carlo_pi_ = std::make_shared<MonteCarloPi<false, false>>(this);
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
  if (monte_carlo_pi_->is_active()) {
    RCLCPP_INFO(this->get_logger(), "Goal rejected: server is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(this->get_logger(), "Goal accepted: server is inactive");
  // set the monte carlo result accept time
  monte_carlo_pi_->set_goal_handle_accept_time(this->now());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// Handle cancel request
rclcpp_action::CancelResponse AnytimeActionServer::handle_cancel(
    const std::shared_ptr<AnytimeGoalHandle> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;  // Suppress unused variable warning

  // Cancel the MonteCarloPi
  monte_carlo_pi_->cancel();

  return rclcpp_action::CancelResponse::ACCEPT;
}

void AnytimeActionServer::handle_accepted(
    const std::shared_ptr<AnytimeGoalHandle> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Activating MonteCarloPi");
  monte_carlo_pi_->activate();

  RCLCPP_INFO(this->get_logger(), "Setting goal handle for MonteCarloPi");
  monte_carlo_pi_->set_goal_handle(goal_handle);

  RCLCPP_INFO(this->get_logger(), "Resetting MonteCarloPi");
  monte_carlo_pi_->reset();

  RCLCPP_INFO(this->get_logger(), "Start MonteCarloPi");
  monte_carlo_pi_->start();
}