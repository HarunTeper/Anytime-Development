#ifndef ANYTIME_TEMPLATE_HPP
#define ANYTIME_TEMPLATE_HPP

#include "anytime_monte_carlo/anytime_waitable.hpp"

template <typename InputType, typename ReturnType, typename GoalHandleType,
          bool isBlocking, bool isActive>
class AnytimeModel {
 public:
  bool canceled_ = false;

  // Constructor that takes an AnytimeWaitable
  AnytimeModel(std::shared_ptr<AnytimeWaitable> waitable)
      : anytime_waitable_(waitable) {}

  // function with input of goal handle
  ReturnType call_blocking(const GoalHandleType& goal_handle) {
    if constexpr (isActive) {
      while (true) {
        if (goal_handle->is_canceling()) {
          goal_handle->canceled();
          return ReturnType();
        }
      }
    }
  }

  // Pure virtual function for blocking operation
  virtual ReturnType blocking_function(const InputType& input) = 0;

  // Pure virtual function for non-blocking operation
  virtual void non_blocking_function(const InputType& input,
                                     std::shared_ptr<ReturnType> result) = 0;

  std::shared_ptr<AnytimeWaitable> anytime_waitable_;
};
;

#endif  // ANYTIME_TEMPLATE_HPP