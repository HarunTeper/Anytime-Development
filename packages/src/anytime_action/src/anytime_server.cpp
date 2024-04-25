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
    auto & feedback_value = feedback->feedback;
    feedback_value = 0;
    auto result = std::make_shared<anytime_interfaces::action::Anytime::Result>();
    result->result = 0;

    int count_total = 0;
    int count_inside = 0;
    int count_outside = 0;

    float_t x = 0.0;
    float_t y = 0.0;

    for (int i = 1; i <= goal_handle->get_goal()->goal; i++){
        if (goal_handle->is_canceling()){
            RCLCPP_INFO(this->get_logger(), "Goal was canceled");
            result->result = feedback_value;
            goal_handle->canceled(result);
            return;
        }
         // sample x and y between 0 and 1, and if the length of the vector is greater than one, add count to count_outside, otherwise add to count_inside
        x = (float_t)rand() / RAND_MAX;
        y = (float_t)rand() / RAND_MAX;

        if (sqrt(pow(x, 2) + pow(y, 2)) <= 1){
            count_inside++;
        } else {
            count_outside++;
        }
        count_total++;

        feedback_value = 4 * (float_t)count_inside / count_total;

        goal_handle->publish_feedback(feedback);

    }

    result->result = feedback_value;

    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "Goal was completed");
}

void AnytimeActionServer::handle_accepted(const std::shared_ptr<AnytimeGoalHandle> goal_handle){
    std::thread{std::bind(&AnytimeActionServer::execute, this, goal_handle)}.detach();
}