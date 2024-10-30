#ifndef ANYTIME_TEMPLATE_HPP
#define ANYTIME_TEMPLATE_HPP

#include "anytime_monte_carlo/anytime_waitable.hpp"

template <typename ReturnType, typename InterfaceType, typename GoalHandleType,
          bool isBlocking, bool isActive>
class AnytimeModel {
 public:
  bool canceled_ = false;

  // Constructor that takes an AnytimeWaitable
  AnytimeModel() {
    anytime_waitable_ = std::make_shared<AnytimeWaitable>(
        [this]() { this->blocking_function(goal_handle_); });
  }

  // // function with input of goal handle
  // ReturnType call_blocking(const GoalHandleType& goal_handle) {
  //   if constexpr (isActive) {
  //     while (true) {
  //       if (goal_handle->is_canceling()) {
  //         goal_handle->canceled();
  //         return ReturnType();
  //       }
  //     }
  //   }
  // }

  // void cancel() {
  //   if constexpr (isActive) {
  //     goal_handle_->succeed(result_);
  //   }
  // }

  void activate() { is_running_ = true; }

  void deactivate() { is_running_ = false; }

  bool is_active() { return is_running_; }

  virtual void reset() = 0;

  // Pure virtual function for blocking operation
  virtual void blocking_function(
      const std::shared_ptr<GoalHandleType> goal_handle) = 0;

  // Pure virtual function for non-blocking operation
  virtual void non_blocking_function(
      const std::shared_ptr<GoalHandleType> goal_handle,
      std::shared_ptr<ReturnType> result) = 0;

  virtual bool check_cancel() = 0;

  virtual void finish() = 0;

  std::shared_ptr<AnytimeWaitable> anytime_waitable_;

  // goal handle
  std::shared_ptr<GoalHandleType> goal_handle_;

  std::shared_ptr<typename InterfaceType::Feedback> feedback_ =
      std::make_shared<typename InterfaceType::Feedback>();

  std::shared_ptr<typename InterfaceType::Result> result_ =
      std::make_shared<typename InterfaceType::Result>();

  bool is_running_ = false;
};

#endif  // ANYTIME_TEMPLATE_HPP