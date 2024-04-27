#include "anytime_action/anytime_client.hpp"

AnytimeActionClient::AnytimeActionClient(const rclcpp::NodeOptions & options) : Node("anytime_action_client", options)
{
    RCLCPP_INFO(this->get_logger(), "Starting Anytime action client");
    action_client_ = rclcpp_action::create_client<anytime_interfaces::action::Anytime>(this, "anytime");

    timer_ = this->create_wall_timer(std::chrono::seconds(5), [this]() {this->send_goal();});
    goal_handle_timer_ = this->create_wall_timer(std::chrono::seconds(0), [this]() {this->receive_goal_handle();});
    goal_handle_timer_->cancel();
    result_timer_ = this->create_wall_timer(std::chrono::seconds(0), [this]() {this->receive_result();});
    result_timer_->cancel();
    cancel_timer_ = this->create_wall_timer(std::chrono::seconds(0), [this]() {this->cancel_goal();});
    cancel_timer_->cancel();
    cancel_timeout_timer_ = this->create_wall_timer(std::chrono::seconds(3), [this]() {this->cancel_timeout_callback();});
    cancel_timeout_timer_->cancel();

    timeout_ = 3;
}

AnytimeActionClient::~AnytimeActionClient()
{
}

void AnytimeActionClient::send_goal()
{
    timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto goal_msg = anytime_interfaces::action::Anytime::Goal();
    goal_msg.goal = 5000000;

    auto send_goal_options = rclcpp_action::Client<anytime_interfaces::action::Anytime>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](AnytimeGoalHandle::SharedPtr goal_handle) {this->goal_response_callback(goal_handle);};
    send_goal_options.feedback_callback = [this](AnytimeGoalHandle::SharedPtr goal_handle, const std::shared_ptr<const anytime_interfaces::action::Anytime::Feedback> feedback) {this->feedback_callback(goal_handle, feedback);};
    send_goal_options.result_callback = [this](const AnytimeGoalHandle::WrappedResult & result) {this->result_callback(result);};

    goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);

    goal_handle_timer_->reset();
    return;
}

void AnytimeActionClient::receive_goal_handle()
{
    RCLCPP_INFO(this->get_logger(), "Waiting for goal to be accepted");

    if(goal_handle_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
        goal_handle_timer_->cancel();
        if((goal_handle_ = goal_handle_future_.get()) == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, sending result request");

        cancel_timeout_timer_->reset();

        // result_future_ = action_client_->async_get_result(goal_handle_);

        // RCLCPP_INFO(this->get_logger(), "Send result request");
        // result_timer_->reset();
        // start_time_ = this->now();
    }
}

void AnytimeActionClient::cancel_timeout_callback()
{
    RCLCPP_INFO(this->get_logger(), "Timeout reached, canceling goal");

    cancel_timeout_timer_->cancel();

    cancel_future_ = action_client_->async_cancel_goal(goal_handle_);

    RCLCPP_INFO(this->get_logger(), "Send cancel request");

    timer_->reset();
}

// NOT USED ANYMORE
void AnytimeActionClient::receive_result()
{
    RCLCPP_INFO(this->get_logger(), "Waiting for result");

    if(result_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
        RCLCPP_INFO(this->get_logger(), "Result received");
        result_timer_->cancel();
        rclcpp_action::ClientGoalHandle<anytime_interfaces::action::Anytime>::WrappedResult wrapped_result = result_future_.get();
        switch(wrapped_result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Result received: %f", wrapped_result.result->result);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                RCLCPP_INFO(this->get_logger(), "Result after cancel: %f", wrapped_result.result->result);
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
        }
    }
    else if((this->now() - start_time_) > std::chrono::seconds(timeout_))
    {
        RCLCPP_INFO(this->get_logger(), "Canceling goal");

        result_timer_->cancel();

        cancel_future_ = action_client_->async_cancel_goal(goal_handle_, [this](std::shared_ptr<action_msgs::srv::CancelGoal::Response> cancel_result) {this->cancel_response_callback(cancel_result);});
        // cancel_timer_->reset();
        return;
    }
}

// NOT USED ANYMORE
void AnytimeActionClient::cancel_response_callback(std::shared_ptr<action_msgs::srv::CancelGoal::Response> cancel_result)
{
    if(cancel_result->return_code == action_msgs::srv::CancelGoal::Response::ERROR_REJECTED)
    {
        RCLCPP_ERROR(this->get_logger(), "Error canceling goal");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Goal canceled");

        rclcpp_action::ClientGoalHandle<anytime_interfaces::action::Anytime>::WrappedResult wrapped_result = result_future_.get();
        switch(wrapped_result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Result received: %f", wrapped_result.result->result);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                RCLCPP_INFO(this->get_logger(), "Result after cancel: %f", wrapped_result.result->result);
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
        }
    }
}

// NOT USED ANYMORE
void AnytimeActionClient::cancel_goal()
{
    RCLCPP_INFO(this->get_logger(), "Cancelling goal");
    if(cancel_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
        //print the value of the result from the goal handle
        // RCLCPP_INFO(this->get_logger(), "Result received: %f", cancel_future_.get()->result->result);


        cancel_timer_->cancel();
        goal_handle_.reset();
        return;
    }
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
    RCLCPP_INFO(this->get_logger(), "Next number in the sequence: %f", feedback->feedback);
}

void AnytimeActionClient::result_callback(const AnytimeGoalHandle::WrappedResult & result)
{
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Result received: %f", result.result->result);
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            RCLCPP_INFO(this->get_logger(), "Result after cancel callback: %f", result.result->result);
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
    }
}