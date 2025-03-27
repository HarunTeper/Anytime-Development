#ifndef ANYTIME_BASE_HPP
#define ANYTIME_BASE_HPP

#include "anytime_monte_carlo/anytime_waitable.hpp"

#include <sys/types.h>

template <typename ReturnType, typename InterfaceType, typename GoalHandleType>
class AnytimeBase
{
public:
  virtual ~AnytimeBase() = default;

  virtual void start() = 0;
  virtual void compute() = 0;
  virtual void notify_cancel() = 0;
  virtual void calculate_result() = 0;
  virtual void check_cancel_and_finish_proactive() = 0;
  virtual bool check_cancel_and_finish_reactive() = 0;
  virtual void reset() = 0;

  void activate() { is_running_ = true; }
  void deactivate() { is_running_ = false; }
  bool is_running() { return is_running_; }

  void set_goal_handle(std::shared_ptr<GoalHandleType> goal_handle) { goal_handle_ = goal_handle; }

  std::shared_ptr<GoalHandleType> get_goal_handle() { return goal_handle_; }

  void notify_iteration() { anytime_iteration_waitable_->notify(); }

  void notify_result() { anytime_result_waitable_->notify(); }

  void notify_check_finish() { anytime_check_finish_waitable_->notify(); }

  void set_goal_handle_receive_time(rclcpp::Time time) { server_goal_receive_time_ = time; }
  void set_goal_handle_accept_time(rclcpp::Time time) { server_goal_accept_time_ = time; }
  void set_goal_processing_start_time(rclcpp::Time time) { server_goal_start_time_ = time; }

protected:
  std::shared_ptr<AnytimeWaitable> anytime_iteration_waitable_;
  std::shared_ptr<AnytimeWaitable> anytime_result_waitable_;
  std::shared_ptr<AnytimeWaitable> anytime_check_finish_waitable_;

  bool is_running_ = false;

  // goal handle
  std::shared_ptr<GoalHandleType> goal_handle_;

  std::shared_ptr<typename InterfaceType::Feedback> feedback_ =
    std::make_shared<typename InterfaceType::Feedback>();

  std::shared_ptr<typename InterfaceType::Result> result_ =
    std::make_shared<typename InterfaceType::Result>();

  // rclcpp time
  rclcpp::Time server_goal_accept_time_;
  rclcpp::Time server_goal_receive_time_;
  rclcpp::Time server_goal_start_time_;

  double average_computation_time_ = 0.0;  // Store in milliseconds
  uint64_t batch_count_ = 0;

  // callback group for compute
  rclcpp::CallbackGroup::SharedPtr compute_callback_group_;

  int batch_size_ = 1;  // Default batch size
};

#endif  // ANYTIME_BASE_HPP
