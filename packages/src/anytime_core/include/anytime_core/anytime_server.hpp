#ifndef ANYTIME_CORE_ANYTIME_SERVER_HPP
#define ANYTIME_CORE_ANYTIME_SERVER_HPP

#include "anytime_core/anytime_base.hpp"
#include "anytime_core/tracing.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <memory>

namespace anytime_core
{

template <typename ActionType>
class AnytimeActionServerBase : public rclcpp::Node
{
public:
  using GoalHandleType = rclcpp_action::ServerGoalHandle<ActionType>;

  explicit AnytimeActionServerBase(
    const std::string & node_name, rclcpp::NodeOptions options = rclcpp::NodeOptions())
  : Node(node_name, options)
  {
    TRACE_ANYTIME_SERVER_INIT(this, node_name.c_str());

    // Create the action server - common for all derived classes
    action_server_ = rclcpp_action::create_server<ActionType>(
      this, "anytime",
      [this](
        const rclcpp_action::GoalUUID uuid, std::shared_ptr<const typename ActionType::Goal> goal) {
        return this->handle_goal(uuid, goal);
      },
      [this](const std::shared_ptr<GoalHandleType> goal_handle) {
        return this->handle_cancel(goal_handle);
      },
      [this](const std::shared_ptr<GoalHandleType> goal_handle) {
        return this->handle_accepted(goal_handle);
      },
      rcl_action_server_get_default_options(),
      this->get_node_base_interface()->get_default_callback_group());
  }

  virtual ~AnytimeActionServerBase() = default;

protected:
  rclcpp_action::Server<ActionType>::SharedPtr action_server_;
  std::shared_ptr<AnytimeBase<ActionType, GoalHandleType>> anytime_management_;

  // Common goal handling
  virtual rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const typename ActionType::Goal> goal)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received goal request");
    (void)uuid;  // Suppress unused variable warning
    (void)goal;  // Suppress unused variable warning

    bool accepted = true;
    if (anytime_management_->is_running()) {
      RCLCPP_DEBUG(this->get_logger(), "Goal rejected: server is busy");
      accepted = false;
      TRACE_ANYTIME_SERVER_HANDLE_GOAL(this, accepted);
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_DEBUG(this->get_logger(), "Goal accepted: server is inactive");
    TRACE_ANYTIME_SERVER_HANDLE_GOAL(this, accepted);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Common cancel handling
  virtual rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleType> goal_handle)
  {
    TRACE_ANYTIME_SERVER_HANDLE_CANCEL(this);
    RCLCPP_DEBUG(
      this->get_logger(), "[Goal ID: %s] Received cancel request",
      rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
    anytime_management_->notify_cancel();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Common accepted handling
  virtual void handle_accepted(const std::shared_ptr<GoalHandleType> goal_handle)
  {
    TRACE_ANYTIME_SERVER_HANDLE_ACCEPTED(this);
    std::string goal_id_str = rclcpp_action::to_string(goal_handle->get_goal_id());
    RCLCPP_DEBUG(
      this->get_logger(), "[Goal ID: %s] Setting goal handle for AnytimeManagement",
      goal_id_str.c_str());
    anytime_management_->set_goal_handle(goal_handle);

    RCLCPP_DEBUG(
      this->get_logger(), "[Goal ID: %s] Resetting AnytimeManagement", goal_id_str.c_str());
    anytime_management_->reset();

    RCLCPP_DEBUG(
      this->get_logger(), "[Goal ID: %s] Activating AnytimeManagement", goal_id_str.c_str());
    anytime_management_->activate();

    RCLCPP_DEBUG(this->get_logger(), "[Goal ID: %s] Start AnytimeManagement", goal_id_str.c_str());
    anytime_management_->start();
  }
};

}  // namespace anytime_core

#endif  // ANYTIME_CORE_ANYTIME_SERVER_HPP
