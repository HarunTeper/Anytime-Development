#include "anytime_yolo/anytime_server.hpp"

#include <rclcpp/logging.hpp>

#include <map>

// Constructor for the AnytimeActionServer class
AnytimeActionServer::AnytimeActionServer(rclcpp::NodeOptions options)
: Node("anytime_action_server", options.use_intra_process_comms(true))
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
  std::string single_multi_str = this->declare_parameter("is_single_multi", "single");
  int batch_size = this->declare_parameter("batch_size", 1);
  std::string weights_path = this->declare_parameter("weights_path", "");

  // Convert strings to booleans
  bool is_reactive_proactive = (reactive_proactive_str == "proactive");
  bool is_single_multi = (single_multi_str == "multi");

  RCLCPP_INFO(
    this->get_logger(), "is_reactive_proactive: %s",
    is_reactive_proactive ? "proactive" : "reactive");
  RCLCPP_INFO(
    this->get_logger(), "is_single_multi: %s",
    is_single_multi ? "multi-threaded" : "single-threaded");
  RCLCPP_INFO(this->get_logger(), "batch_size: %d", batch_size);
  RCLCPP_INFO(this->get_logger(), "weights_path: %s", weights_path.c_str());

  // Create the MonteCarloPi instance using a factory function
  anytime_management_ = create_anytime_management(
    this, is_reactive_proactive, is_single_multi, batch_size, weights_path);
}

// Destructor for the AnytimeActionServer class
AnytimeActionServer::~AnytimeActionServer() {}

// factory function
std::shared_ptr<AnytimeBase<double, Anytime, AnytimeGoalHandle>>
AnytimeActionServer::create_anytime_management(
  rclcpp::Node * node, bool is_reactive_proactive, bool is_single_multi, int batch_size,
  const std::string & weights_path)
{
  if (is_reactive_proactive) {
    if (is_single_multi) {
      return std::make_shared<AnytimeManagement<true, true>>(node, batch_size, weights_path);
    } else {
      return std::make_shared<AnytimeManagement<true, false>>(node, batch_size, weights_path);
    }
  } else {
    if (is_single_multi) {
      return std::make_shared<AnytimeManagement<false, true>>(node, batch_size, weights_path);
    } else {
      return std::make_shared<AnytimeManagement<false, false>>(node, batch_size, weights_path);
    }
  }
}

// Handle goal request
rclcpp_action::GoalResponse AnytimeActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid, const std::shared_ptr<const Anytime::Goal> goal)
{
  anytime_management_->set_goal_handle_receive_time(this->now());
  RCLCPP_DEBUG(this->get_logger(), "Received goal request");
  (void)uuid;  // Suppress unused variable warning
  if (anytime_management_->is_running()) {
    RCLCPP_DEBUG(this->get_logger(), "Goal rejected: server is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_DEBUG(this->get_logger(), "Goal accepted: server is inactive");
  // set the monte carlo result accept time
  anytime_management_->set_goal_handle_accept_time(this->now());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// Handle cancel request
rclcpp_action::CancelResponse AnytimeActionServer::handle_cancel(
  const std::shared_ptr<AnytimeGoalHandle> goal_handle)
{
  RCLCPP_DEBUG(this->get_logger(), "Received cancel request");
  (void)goal_handle;  // Suppress unused variable warning

  // Cancel the MonteCarloPi
  anytime_management_->notify_cancel();

  return rclcpp_action::CancelResponse::ACCEPT;
}

void AnytimeActionServer::handle_accepted(const std::shared_ptr<AnytimeGoalHandle> goal_handle)
{
  RCLCPP_DEBUG(this->get_logger(), "Setting goal handle for MonteCarloPi");
  anytime_management_->set_goal_handle(goal_handle);

  RCLCPP_DEBUG(this->get_logger(), "Resetting MonteCarloPi");
  anytime_management_->reset();

  RCLCPP_DEBUG(this->get_logger(), "Activating MonteCarloPi");
  anytime_management_->activate();

  RCLCPP_DEBUG(this->get_logger(), "Start MonteCarloPi");

  anytime_management_->set_goal_processing_start_time(this->now());
  // anytime_management_->start();
}