#include "anytime_yolo/anytime_server.hpp"

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <string>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<AnytimeActionServer>(options);

  // Default to single-threaded
  std::string executor_type = "single";

  // Parse command-line args
  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg == "--is_single_multi" && i + 1 < argc) {
      executor_type = argv[i + 1];
      i++;  // Skip the next argument since we consumed it
    }
  }

  if (executor_type == "multi") {
    // Use multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } else {
    // Use single-threaded executor (default)
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }

  rclcpp::shutdown();
  return 0;
}
