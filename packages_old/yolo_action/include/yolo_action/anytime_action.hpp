#include "rclcpp/rclcpp.hpp"

class AnytimeST : public rclcpp::Node {
 public:
  AnytimeST();
  ~AnytimeST();

 private:
  // create a std::thread
  void resource_thread();
  std::thread resource_thread_;
};

class AnytimeMT : public rclcpp::Node {
 public:
  AnytimeMT();

 private:
};