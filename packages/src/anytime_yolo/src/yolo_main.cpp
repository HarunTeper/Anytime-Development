#include "anytime_yolo/yolo_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::cout << "Single threaded" << std::endl;
  auto node = std::make_shared<YOLONode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}