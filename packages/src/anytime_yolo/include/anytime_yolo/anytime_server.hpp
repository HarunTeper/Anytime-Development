#include "anytime_core/anytime_server.hpp"
#include "anytime_interfaces/action/yolo.hpp"
#include "anytime_yolo/anytime_management.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class AnytimeActionServer
: public anytime_core::AnytimeActionServerBase<anytime_interfaces::action::Yolo>
{
public:
  using Anytime = anytime_interfaces::action::Yolo;
  using GoalHandleType = rclcpp_action::ServerGoalHandle<Anytime>;

  explicit AnytimeActionServer(rclcpp::NodeOptions options);
  ~AnytimeActionServer() override;

private:
  // factory function for creating anytime management
  std::shared_ptr<anytime_core::AnytimeBase<Anytime, GoalHandleType>> create_anytime_management(
    rclcpp::Node * node, bool is_reactive_proactive, bool is_sync_async, int block_size,
    const std::string & weights_path);
};