#include "anytime_yolo/anytime_client.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<AnytimeActionClient>(options);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
