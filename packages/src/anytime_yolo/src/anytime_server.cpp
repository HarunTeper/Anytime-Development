#include "anytime_yolo/anytime_server.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#include <rclcpp/logging.hpp>

#include <map>

// Constructor for the AnytimeActionServer class
AnytimeActionServer::AnytimeActionServer(rclcpp::NodeOptions options)
: anytime_core::AnytimeActionServerBase<Anytime>(
    "anytime_action_server", options.use_intra_process_comms(true))
{
  RCLCPP_DEBUG(this->get_logger(), "Starting Anytime action server");

  // Read the ros2 parameters
  std::string reactive_proactive_str = this->declare_parameter("is_reactive_proactive", "reactive");
  std::string sync_async_str = this->declare_parameter("is_sync_async", "sync");
  int block_size = this->declare_parameter("block_size", 1);
  std::string weights_path = this->declare_parameter("weights_path", "");

  // Convert strings to booleans
  bool is_reactive_proactive = (reactive_proactive_str == "proactive");
  bool is_sync_async = (sync_async_str == "async");

  RCLCPP_INFO(this->get_logger(), "YOLO Action Server initialized with parameters:");
  RCLCPP_INFO(this->get_logger(), "  is_reactive_proactive: %s", reactive_proactive_str.c_str());
  RCLCPP_INFO(this->get_logger(), "  is_sync_async: %s", sync_async_str.c_str());
  RCLCPP_INFO(this->get_logger(), "  block_size: %d", block_size);
  RCLCPP_INFO(this->get_logger(), "  weights_path: %s", weights_path.c_str());

  RCLCPP_DEBUG(
    this->get_logger(), "is_reactive_proactive: %s",
    is_reactive_proactive ? "proactive" : "reactive");
  RCLCPP_DEBUG(this->get_logger(), "is_sync_async: %s", is_sync_async ? "async" : "sync");
  RCLCPP_DEBUG(this->get_logger(), "block_size: %d", block_size);
  RCLCPP_DEBUG(this->get_logger(), "weights_path: %s", weights_path.c_str());

  this->anytime_management_ =
    create_anytime_management(this, is_reactive_proactive, is_sync_async, block_size, weights_path);
}

// Destructor for the AnytimeActionServer class
AnytimeActionServer::~AnytimeActionServer() {}

// factory function
std::shared_ptr<
  anytime_core::AnytimeBase<AnytimeActionServer::Anytime, AnytimeActionServer::GoalHandleType>>
AnytimeActionServer::create_anytime_management(
  rclcpp::Node * node, bool is_reactive_proactive, bool is_sync_async, int block_size,
  const std::string & weights_path)
{
  if (is_reactive_proactive) {
    if (is_sync_async) {
      return std::make_shared<AnytimeManagement<true, true>>(node, block_size, weights_path);
    } else {
      return std::make_shared<AnytimeManagement<true, false>>(node, block_size, weights_path);
    }
  } else {
    if (is_sync_async) {
      return std::make_shared<AnytimeManagement<false, true>>(node, block_size, weights_path);
    } else {
      return std::make_shared<AnytimeManagement<false, false>>(node, block_size, weights_path);
    }
  }
}

// Register the component
RCLCPP_COMPONENTS_REGISTER_NODE(AnytimeActionServer)
