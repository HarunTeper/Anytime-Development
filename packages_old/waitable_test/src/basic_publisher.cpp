#include "waitable_test/basic_publisher.hpp"

BasicPublisher::BasicPublisher() : rclcpp::Node("basic_publisher_node") {
  timer = this->create_wall_timer(std::chrono::milliseconds(1000),
                                  [this]() { timer_callback(); });

  cuda_waitable_ =
      std::make_shared<CudaWaitable>([this]() { guard_condition_callback(); });

  this->get_node_waitables_interface()->add_waitable(
      cuda_waitable_,
      this->get_node_base_interface()->get_default_callback_group());
}

void BasicPublisher::timer_callback() {
  RCLCPP_INFO(this->get_logger(), "Hello, world!");
  cuda_waitable_->notify();
}

void BasicPublisher::guard_condition_callback() {
  RCLCPP_INFO(this->get_logger(), "Hello, guard condition!");
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BasicPublisher>());
  rclcpp::shutdown();
  return 0;
}