#include "anytime_action/anytime_client.hpp"

AnytimeActionClient::AnytimeActionClient(const rclcpp::NodeOptions & options) : Node("anytime_action_client", options)
{
    RCLCPP_INFO(this->get_logger(), "Starting Anytime action client");
    action_client_ = rclcpp_action::create_client<anytime_interfaces::action::Anytime>(this, "anytime");

    timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {this->send_goal();});

}

AnytimeActionClient::~AnytimeActionClient()
{
}

void AnytimeActionClient::send_goal()
{
    RCLCPP_INFO(this->get_logger(), "Sending goal");
    this->timer_->cancel();

    auto goal_msg = anytime_interfaces::action::Anytime::Goal();
    goal_msg.goal = 1;

    auto send_goal_options = rclcpp_action::Client<anytime_interfaces::action::Anytime>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](AnytimeGoalHandle::SharedPtr goal_handle) {this->goal_response_callback(goal_handle);};
    send_goal_options.feedback_callback = [this](AnytimeGoalHandle::SharedPtr goal_handle, const std::shared_ptr<const anytime_interfaces::action::Anytime::Feedback> feedback) {this->feedback_callback(goal_handle, feedback);};
    send_goal_options.result_callback = [this](const AnytimeGoalHandle::WrappedResult & result) {this->result_callback(result);};

    action_client_->async_send_goal(goal_msg, send_goal_options);
}

void AnytimeActionClient::goal_response_callback(AnytimeGoalHandle::SharedPtr goal_handle)
{
    if (!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        this->timer_.reset();
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void AnytimeActionClient::feedback_callback(AnytimeGoalHandle::SharedPtr, const std::shared_ptr<const anytime_interfaces::action::Anytime::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), "Next number in the sequence: %d", feedback->feedback);
}

void AnytimeActionClient::result_callback(const AnytimeGoalHandle::WrappedResult & result)
{
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Result received: %d", result.result->result);
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
    }
}