#include "rclcpp/rclcpp.hpp"

class AnytimeST : public rclcpp::Node {
 public:
  AnytimeST();

 private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time start_time;
  rclcpp::Time end_time;

  void timer_callback();
};

class AnytimeMT : public rclcpp::Node {
 public:
  AnytimeMT();

 private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time start_time;
  rclcpp::Time end_time;

  void timer_callback();
};