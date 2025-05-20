#include "anytime_interfaces/action/monte_carlo.hpp"
#include "anytime_monte_carlo/anytime_management.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class AnytimeActionServer : public rclcpp::Node
{
public:
  AnytimeActionServer(rclcpp::NodeOptions options = rclcpp::NodeOptions());
  ~AnytimeActionServer();

  using Anytime = anytime_interfaces::action::MonteCarlo;
  using AnytimeGoalHandle = rclcpp_action::ServerGoalHandle<Anytime>;

private:
  rclcpp_action::Server<Anytime>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Anytime::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<AnytimeGoalHandle> goal_handle);

  void handle_accepted(const std::shared_ptr<AnytimeGoalHandle> goal_handle);

  void execute(const std::shared_ptr<AnytimeGoalHandle> goal_handle);

  // factory function for creating anytime management
  std::shared_ptr<AnytimeBase<double, Anytime, AnytimeGoalHandle>> create_anytime_management(
    rclcpp::Node * node, bool is_reactive_proactive, int batch_size);

  std::shared_ptr<AnytimeBase<double, Anytime, AnytimeGoalHandle>> anytime_management_;
};  // Added missing semicolon here