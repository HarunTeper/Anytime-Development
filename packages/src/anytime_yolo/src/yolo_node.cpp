#include "anytime_yolo/yolo_node.hpp"

#include "anytime_yolo/yolo_model.hpp"

YOLONode::YOLONode() : Node("YOLO")
{
  RCLCPP_INFO(get_logger(), "YOLONode");
  return;
}
