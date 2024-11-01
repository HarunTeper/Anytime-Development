#include "anytime_monte_carlo/anytime_server.hpp"
#include <map>
#include <rclcpp/logging.hpp>

// Constructor for the AnytimeActionServer class
AnytimeActionServer::AnytimeActionServer(rclcpp::NodeOptions options)
    : Node("anytime_action_server", options.use_intra_process_comms(true)) {
  RCLCPP_INFO(this->get_logger(), "Starting Anytime action server");

  // Create the action server
  // Create a callback group for the action server
  // callback_group_ =_group(rclcpp::CallbackGroupType::MutuallyExclusive);

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
  bool separate_thread = this->declare_parameter("separate_thread", false);
  bool multi_threading = this->declare_parameter("multi_threading", false);

  RCLCPP_INFO(this->get_logger(), "anytime_active: %s",
              anytime_active ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "separate_thread: %s",
              separate_thread ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "multi_threading: %s",
              multi_threading ? "true" : "false");

  // Create the MonteCarloPi instance using a factory function
  monte_carlo_pi_ = create_monte_carlo_pi(this, anytime_active, separate_thread,
                                          multi_threading);
}

// Destructor for the AnytimeActionServer class
AnytimeActionServer::~AnytimeActionServer() {}

// factory function
std::shared_ptr<AnytimeBase<double, Anytime, AnytimeGoalHandle>>
AnytimeActionServer::create_monte_carlo_pi(rclcpp::Node* node,
                                           bool anytime_active,
                                           bool separate_thread,
                                           bool multi_threading) {
  if (anytime_active) {
    if (separate_thread) {
      if (multi_threading) {
        return std::make_shared<MonteCarloPi<true, true, true>>(node);
      } else {
        return std::make_shared<MonteCarloPi<true, true, false>>(node);
      }
    } else {
      if (multi_threading) {
        return std::make_shared<MonteCarloPi<true, false, true>>(node);
      } else {
        return std::make_shared<MonteCarloPi<true, false, false>>(node);
      }
    }
  } else {
    if (separate_thread) {
      if (multi_threading) {
        return std::make_shared<MonteCarloPi<false, true, true>>(node);
      } else {
        return std::make_shared<MonteCarloPi<false, true, false>>(node);
      }
    } else {
      if (multi_threading) {
        return std::make_shared<MonteCarloPi<false, false, true>>(node);
      } else {
        return std::make_shared<MonteCarloPi<false, false, false>>(node);
      }
    }
  }
}

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
  RCLCPP_INFO(this->get_logger(), "Setting goal handle for MonteCarloPi");
  monte_carlo_pi_->set_goal_handle(goal_handle);

  RCLCPP_INFO(this->get_logger(), "Resetting MonteCarloPi");
  monte_carlo_pi_->reset();

  RCLCPP_INFO(this->get_logger(), "Activating MonteCarloPi");
  monte_carlo_pi_->activate();

  RCLCPP_INFO(this->get_logger(), "Start MonteCarloPi");
  monte_carlo_pi_->start();
}