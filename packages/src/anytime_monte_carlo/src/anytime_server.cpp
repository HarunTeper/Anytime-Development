#include "anytime_monte_carlo/anytime_server.hpp"

#include <rclcpp/logging.hpp>

#include <map>

// Constructor for the AnytimeActionServer class
AnytimeActionServer::AnytimeActionServer(rclcpp::NodeOptions options)
: Node("anytime_action_server", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting Anytime action server");

  // Create the action server
  action_server_ = rclcpp_action::create_server<Anytime>(
    this, "anytime",
    // Lambda function to handle goal requests
    [this](const rclcpp_action::GoalUUID uuid, std::shared_ptr<const Anytime::Goal> goal) {
      return this->handle_goal(uuid, goal);
    },
    // Lambda function to handle cancel requests
    [this](const std::shared_ptr<AnytimeGoalHandle> goal_handle) {
      return this->handle_cancel(goal_handle);
    },
    // Lambda function to handle accepted goals
    [this](const std::shared_ptr<AnytimeGoalHandle> goal_handle) {
      return this->handle_accepted(goal_handle);
    });

  // Read the ros2 parameters
  std::string reactive_proactive_str = this->declare_parameter("is_reactive_proactive", "reactive");
  int batch_size = this->declare_parameter("batch_size", 1);

  // Convert strings to booleans
  bool is_reactive_proactive = (reactive_proactive_str == "proactive");

  RCLCPP_INFO(
    this->get_logger(), "is_reactive_proactive: %s",
    is_reactive_proactive ? "proactive" : "reactive");
  RCLCPP_INFO(this->get_logger(), "batch_size: %d", batch_size);

  // Create the Anytime management object based on the parameters
  anytime_management_ = create_anytime_management(this, is_reactive_proactive, batch_size);
}

// Destructor for the AnytimeActionServer class
AnytimeActionServer::~AnytimeActionServer() {}

// factory function
std::shared_ptr<AnytimeBase<double, Anytime, AnytimeGoalHandle>>
AnytimeActionServer::create_anytime_management(
  rclcpp::Node * node, bool is_reactive_proactive, int batch_size)
{
  if (is_reactive_proactive) {
    return std::make_shared<AnytimeManagement<true>>(node, batch_size);
  } else {
    return std::make_shared<AnytimeManagement<false>>(node, batch_size);
  }
}

// Handle goal request
rclcpp_action::GoalResponse AnytimeActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid, const std::shared_ptr<const Anytime::Goal> goal)
{
  anytime_management_->set_goal_handle_receive_time(this->now());
  RCLCPP_INFO(this->get_logger(), "Received goal request with number %d", goal->goal);
  (void)uuid;  // Suppress unused variable warning
  if (anytime_management_->is_running()) {
    RCLCPP_ERROR(this->get_logger(), "Goal rejected: server is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(this->get_logger(), "Goal accepted: server is inactive");
  // set the anytime management goal handle accept time
  anytime_management_->set_goal_handle_accept_time(this->now());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// Handle cancel request
rclcpp_action::CancelResponse AnytimeActionServer::handle_cancel(
  const std::shared_ptr<AnytimeGoalHandle> goal_handle)
{
  anytime_management_->set_goal_handle_cancel_time(this->now());
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  (void)goal_handle;  // Suppress unused variable warning

  // Cancel the Anytime management
  anytime_management_->notify_cancel();

  return rclcpp_action::CancelResponse::ACCEPT;
}

void AnytimeActionServer::handle_accepted(const std::shared_ptr<AnytimeGoalHandle> goal_handle)
{
  anytime_management_->set_goal_processing_start_time(this->now());
  RCLCPP_INFO(this->get_logger(), "Setting goal handle for AnytimeManagement");
  anytime_management_->set_goal_handle(goal_handle);

  RCLCPP_INFO(this->get_logger(), "Resetting AnytimeManagement");
  anytime_management_->reset();

  RCLCPP_INFO(this->get_logger(), "Activating AnytimeManagement");
  anytime_management_->activate();

  RCLCPP_INFO(this->get_logger(), "Start AnytimeManagement");
  anytime_management_->start();
}