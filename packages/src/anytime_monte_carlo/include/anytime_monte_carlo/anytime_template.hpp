#ifndef ANYTIME_TEMPLATE_HPP
#define ANYTIME_TEMPLATE_HPP

#include "anytime_monte_carlo/anytime_waitable.hpp"

template <typename InputType, typename ReturnType, typename InterfaceType,
          typename GoalHandleType, bool isBlocking, bool isActive>
class AnytimeModel {
 public:
  bool canceled_ = false;

  // Constructor that takes an AnytimeWaitable
  AnytimeModel(std::shared_ptr<AnytimeWaitable> waitable)
      : anytime_waitable_(waitable) {
    // TODO create waitable with executing callback
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

  // Pure virtual function for blocking operation
  virtual ReturnType blocking_function(const InputType& input) = 0;

  // Pure virtual function for non-blocking operation
  virtual void non_blocking_function(const InputType& input,
                                     std::shared_ptr<ReturnType> result) = 0;

  std::shared_ptr<AnytimeWaitable> anytime_waitable_;

  // goal handle
  std::shared_ptr<GoalHandleType> goal_handle_;

  std::shared_ptr<typename InterfaceType::Feedback> feedback_ =
      std::make_shared<typename InterfaceType::Feedback>();

  std::shared_ptr<typename InterfaceType::Result> result_ =
      std::make_shared<typename InterfaceType::Result>();
};

#endif  // ANYTIME_TEMPLATE_HPP