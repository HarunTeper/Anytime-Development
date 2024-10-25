#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include "waitable_test/cuda_waitable.hpp"

class BasicPublisher : public rclcpp::Node {
 public:
  BasicPublisher();

 private:
  void timer_callback();
  void guard_condition_callback();

  rclcpp::TimerBase::SharedPtr timer;

  std::shared_ptr<CudaWaitable> cuda_waitable_;
};