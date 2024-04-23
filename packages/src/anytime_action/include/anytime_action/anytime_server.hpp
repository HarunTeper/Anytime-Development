#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "anytime_interfaces/action/anytime.hpp"

class AnytimeActionServer : public rclcpp::Node
{
    public:
        AnytimeActionServer(const rclcpp::NodeOptions & options);
        ~AnytimeActionServer();

        using Anytime = anytime_interfaces::action::Anytime;
        using AnytimeGoalHandle = rclcpp_action::ServerGoalHandle<Anytime>;

    private:
        rclcpp_action::Server<Anytime>::SharedPtr action_server_;

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const Anytime::Goal> goal);

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<AnytimeGoalHandle> goal_handle);

        void execute(const std::shared_ptr<AnytimeGoalHandle> goal_handle);

        void handle_accepted(const std::shared_ptr<AnytimeGoalHandle> goal_handle);
};

RCLCPP_COMPONENTS_REGISTER_NODE(AnytimeActionServer)