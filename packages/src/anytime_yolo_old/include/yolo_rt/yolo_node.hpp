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

  YOLOModelConfig create_config();

  bool use_waitable_ = false;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr waitable_timer_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

  sensor_msgs::msg::Image::SharedPtr subscription_msg;
  sensor_msgs::msg::Image::SharedPtr yolo_msg;

  std::shared_ptr<CudaWaitable> cuda_waitable_;

  void timer_forward_callback();
  void waitable_callback();

  void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Time start_time;
  rclcpp::Time end_time;
  // mean and std times
  std::vector<double> times;
  double mean_time = 0;
  double std_time = 0;
};

// ----------------------------

class YOLONodeMT : public rclcpp::Node {
 public:
  YOLONodeMT();

  void initialize_forward(torch::Tensor x);
  bool forward_one();
  // torch::IValue forward(torch::Tensor x);

 private:
  YOLOModel model;

  YOLOModelConfig create_config();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr fake_sub;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  // void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  // sensor_msgs::msg::Image::SharedPtr subscription_msg;
  // sensor_msgs::msg::Image::SharedPtr yolo_msg;

  rclcpp::CallbackGroup::SharedPtr callback_group1_;
  rclcpp::CallbackGroup::SharedPtr callback_group2_;

  std::shared_ptr<CudaWaitable> cuda_waitable_;

  void camera_callback();

  void timer_forward_callback();

  rclcpp::Time start_time;
  rclcpp::Time end_time;

  // mutex
  std::mutex mtx;

  bool forwarding = false;

  bool start_queued = false;
  bool cancel_queued = false;

  torch::Tensor queued_tensor;
};