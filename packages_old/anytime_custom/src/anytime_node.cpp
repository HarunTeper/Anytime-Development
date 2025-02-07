#include "anytime_custom/anytime_node.hpp"

AnytimeST::AnytimeST() : Node("anytime") {
  RCLCPP_INFO(this->get_logger(), "Initializing AnytimeST");
  // initialize timer
  timer_ = this->create_wall_timer(std::chrono::milliseconds(0),
                                   std::bind(&AnytimeST::timer_callback, this));
}

void AnytimeST::timer_callback() {
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  // start time
  auto start = rclcpp::Clock().now();

  int count_total = 0;
  int count_inside = 0;
  int count_outside = 0;

  float_t intermediate_result;

  float_t x = 0.0;
  float_t y = 0.0;

  for (int i = 1; i <= 5000000; i++) {
    // sample x and y between 0 and 1, and if the length of the vector is
    // greater than one, add count to count_outside, otherwise add to
    // count_inside
    x = (float_t)rand() / RAND_MAX;
    y = (float_t)rand() / RAND_MAX;

    if (sqrt(pow(x, 2) + pow(y, 2)) <= 1) {
      count_inside++;
    } else {
      count_outside++;
    }
    count_total++;

    intermediate_result = 4 * (float_t)count_inside / count_total;
  }

  // print the result
  RCLCPP_INFO(this->get_logger(), "Result: %f", intermediate_result);

  // end time
  auto end = rclcpp::Clock().now();
  // calculate the duration
  auto duration = end - start;
  // print the duration in msec and sec
  RCLCPP_INFO(this->get_logger(), "Duration: %f msec",
              duration.nanoseconds() / 1e6);

  RCLCPP_INFO(this->get_logger(), "Goal was completed");
}

AnytimeMT::AnytimeMT() : Node("anytime") {
  RCLCPP_INFO(this->get_logger(), "Initializing AnytimeMT");
  // initialize timer
  timer_ = this->create_wall_timer(std::chrono::milliseconds(0),
                                   std::bind(&AnytimeMT::timer_callback, this));
  // set start time
  start_time = this->now();
}

void AnytimeMT::timer_callback() {
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  // start time
  auto start = rclcpp::Clock().now();

  int count_total = 0;
  int count_inside = 0;
  int count_outside = 0;

  float_t intermediate_result;

  float_t x = 0.0;
  float_t y = 0.0;

  for (int i = 1; i <= 5000000; i++) {
    // sample x and y between 0 and 1, and if the length of the vector is
    // greater than one, add count to count_outside, otherwise add to
    // count_inside
    x = (float_t)rand() / RAND_MAX;
    y = (float_t)rand() / RAND_MAX;

    if (sqrt(pow(x, 2) + pow(y, 2)) <= 1) {
      count_inside++;
    } else {
      count_outside++;
    }
    count_total++;

    intermediate_result = 4 * (float_t)count_inside / count_total;
  }

  // print the result
  RCLCPP_INFO(this->get_logger(), "Result: %f", intermediate_result);

  // end time
  auto end = rclcpp::Clock().now();
  // calculate the duration
  auto duration = end - start;
  // print the duration in msec and sec
  RCLCPP_INFO(this->get_logger(), "Duration: %f msec",
              duration.nanoseconds() / 1e6);

  RCLCPP_INFO(this->get_logger(), "Goal was completed");
}
