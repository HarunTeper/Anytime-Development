#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "torch/script.h"
#include "torch/torch.h"
#include "yolo_rt/yolo_model.hpp"

class YOLONodeST : public rclcpp::Node {
 public:
  YOLONodeST();

  void initialize_forward(torch::Tensor x);
  bool forward_one();
  torch::IValue forward(torch::Tensor x);

 private:
  YOLOModel model;

  rclcpp::TimerBase::SharedPtr timer_;

  YOLOModelConfig create_config();

  void timer_forward_callback();

  rclcpp::Time start_time;
  rclcpp::Time end_time;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  sensor_msgs::msg::Image::SharedPtr subscription_msg;
  sensor_msgs::msg::Image::SharedPtr yolo_msg;
};

class YOLONodeMT : public rclcpp::Node {
 public:
  YOLONodeMT();

  void initialize_forward(torch::Tensor x);
  bool forward_one();
  // torch::IValue forward(torch::Tensor x);

 private:
  YOLOModel model;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr fake_sub;

  YOLOModelConfig create_config();

  void timer_forward_callback();

  rclcpp::Time start_time;
  rclcpp::Time end_time;

  // mutex
  std::mutex mtx;

  bool forwarding = false;

  bool start_queued = false;
  bool cancel_queued = false;

  torch::Tensor queued_tensor;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  // void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void camera_callback();

  rclcpp::CallbackGroup::SharedPtr callback_group1_;
  rclcpp::CallbackGroup::SharedPtr callback_group2_;

  // sensor_msgs::msg::Image::SharedPtr subscription_msg;
  // sensor_msgs::msg::Image::SharedPtr yolo_msg;
};