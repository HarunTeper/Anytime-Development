#include "anytime_action/anytime_server.hpp"

AnytimeActionServer::AnytimeActionServer(const rclcpp::NodeOptions & options) : Node("anytime_action_server", options)
{
    RCLCPP_INFO(this->get_logger(), "Starting Anytime action server");
    action_server_ = rclcpp_action::create_server<anytime_interfaces::action::Anytime>(
        this,
        "anytime",
        [this](const rclcpp_action::GoalUUID uuid, std::shared_ptr<const anytime_interfaces::action::Anytime::Goal> goal) {return this->handle_goal(uuid, goal);},
        [this](const std::shared_ptr<AnytimeGoalHandle> goal_handle) {return this->handle_cancel(goal_handle);},
        [this](const std::shared_ptr<AnytimeGoalHandle> goal_handle) {return this->handle_accepted(goal_handle);}
    );
}

AnytimeActionServer::~AnytimeActionServer()
{
}

rclcpp_action::GoalResponse AnytimeActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    const std::shared_ptr<const anytime_interfaces::action::Anytime::Goal> goal){
    RCLCPP_INFO(this->get_logger(), "Received goal request with number %d", goal->goal);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AnytimeActionServer::handle_cancel(
    const std::shared_ptr<AnytimeGoalHandle> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void AnytimeActionServer::execute(const std::shared_ptr<AnytimeGoalHandle> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    auto feedback = std::make_shared<anytime_interfaces::action::Anytime::Feedback>();
    auto & sequence = feedback->feedback;
    sequence = 0;
    auto result = std::make_shared<anytime_interfaces::action::Anytime::Result>();
    result->result = 0;
    goal_handle->succeed(result);
    goal_handle->publish_feedback(feedback);
}

void AnytimeActionServer::handle_accepted(const std::shared_ptr<AnytimeGoalHandle> goal_handle){
    std::thread{std::bind(&AnytimeActionServer::execute, this, goal_handle)}.detach();
}