#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "anytime_interfaces/action/anytime.hpp"
#include "yolo_action/yolo_model.hpp"

class AnytimeActionServer : public rclcpp::Node {
 public:
  AnytimeActionServer(const rclcpp::NodeOptions& options);
  ~AnytimeActionServer();

  using Anytime = anytime_interfaces::action::Anytime;
  using AnytimeGoalHandle = rclcpp_action::ServerGoalHandle<Anytime>;

 private:
  rclcpp_action::Server<Anytime>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const Anytime::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<AnytimeGoalHandle> goal_handle);

  void execute(const std::shared_ptr<AnytimeGoalHandle> goal_handle,
               std::shared_ptr<rclcpp::TimerBase> timer);

  void handle_accepted(const std::shared_ptr<AnytimeGoalHandle> goal_handle);

  void execute();

  std::shared_ptr<AnytimeGoalHandle> goal_handle_;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // callback group
  rclcpp::CallbackGroup::SharedPtr callback_group1_;

  // YOLO
  YOLOModel model;
  YOLOModelConfig create_config();
  rclcpp::Time start_time;
  rclcpp::Time end_time;
  bool forward_one();
  void initialize_forward(torch::Tensor x);

  // tensor
  torch::Tensor tensor_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(AnytimeActionServer)