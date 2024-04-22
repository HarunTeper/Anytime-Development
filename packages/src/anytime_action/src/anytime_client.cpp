#include "anytime_action/anytime_client.hpp"

AnytimeActionClient::AnytimeActionClient() : Node("anytime_action_client")
{
    action_client_ = rclcpp_action::create_client<anytime_interfaces::action::Anytime>(this, "anytime");
}

AnytimeActionClient::~AnytimeActionClient()
{
}

void AnytimeActionClient::send_goal()
{
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

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<AnytimeActionClient>();
    action_client->send_goal();
    rclcpp::spin(action_client);
    rclcpp::shutdown();
    return 0;
}

