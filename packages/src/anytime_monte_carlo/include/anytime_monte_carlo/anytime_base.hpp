#ifndef ANYTIME_BASE_HPP
#define ANYTIME_BASE_HPP

#include "anytime_monte_carlo/anytime_waitable.hpp"

template <typename ReturnType, typename InterfaceType, typename GoalHandleType>
class AnytimeBase {
 public:
  virtual ~AnytimeBase() = default;

  virtual void reset() = 0;
  // check_cancel
  // check_finish
  virtual void calculate_result_reactive() = 0;
  virtual void calculate_result() = 0;
  virtual void cancel() = 0;
  virtual void start() = 0;

  void activate() { is_running_ = true; }
  void deactivate() { is_running_ = false; }
  bool is_reactive() { return is_running_; }

  void set_goal_handle(std::shared_ptr<GoalHandleType> goal_handle) {
    goal_handle_ = goal_handle;
  }

  std::shared_ptr<GoalHandleType> get_goal_handle() { return goal_handle_; }

  void notify_iteration() { anytime_iteration_waitable_->notify(); }

  void notify_result() { anytime_result_waitable_->notify(); }

  void notify_finish() { anytime_finish_waitable_->notify(); }

  void set_goal_handle_accept_time(rclcpp::Time time) {
    goal_handle_accept_time_ = time;
  }

  void set_goal_processing_start_time(rclcpp::Time time) {
    goal_processing_start_time_ = time;
  }

  void set_batch_size(int batch_size) { batch_size_ = batch_size; }

 protected:
  std::shared_ptr<AnytimeWaitable> anytime_iteration_waitable_;
  std::shared_ptr<AnytimeWaitable> anytime_result_waitable_;
  std::shared_ptr<AnytimeWaitable> anytime_finish_waitable_;
  bool is_running_ = false;
  bool finished_ = false;
  bool canceled_ = false;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // goal handle
  std::shared_ptr<GoalHandleType> goal_handle_;

  std::shared_ptr<typename InterfaceType::Feedback> feedback_ =
      std::make_shared<typename InterfaceType::Feedback>();

  std::shared_ptr<typename InterfaceType::Result> result_ =
      std::make_shared<typename InterfaceType::Result>();

  // accept rclcpp time
  rclcpp::Time goal_handle_accept_time_;

  // goal_processing_start_time
  rclcpp::Time goal_processing_start_time_;

  // notify mutex
  std::mutex notify_mutex_;

  // callback group for compute
  rclcpp::CallbackGroup::SharedPtr compute_callback_group_;

  int batch_size_ = 1;  // Default batch size
};

#endif  // ANYTIME_BASE_HPP
