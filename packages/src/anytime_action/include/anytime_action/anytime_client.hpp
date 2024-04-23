#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "anytime_interfaces/action/anytime.hpp"

class AnytimeActionClient : public rclcpp::Node
{
    public:
        AnytimeActionClient(const rclcpp::NodeOptions & options);
        ~AnytimeActionClient();

        using Anytime = anytime_interfaces::action::Anytime;
        using AnytimeGoalHandle = rclcpp_action::ClientGoalHandle<Anytime>;

    private:
        rclcpp_action::Client<anytime_interfaces::action::Anytime>::SharedPtr action_client_;

        void send_goal();
        void goal_response_callback(std::shared_ptr<AnytimeGoalHandle> goal_handle);
        void feedback_callback(AnytimeGoalHandle::SharedPtr, const std::shared_ptr<const Anytime::Feedback> feedback);
        void result_callback(const AnytimeGoalHandle::WrappedResult & result);

        rclcpp::TimerBase::SharedPtr timer_;

};

RCLCPP_COMPONENTS_REGISTER_NODE(AnytimeActionClient)