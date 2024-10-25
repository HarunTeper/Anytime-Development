#include "waitable_test/cuda_waitable.hpp"
#include <rcl/wait.h>

CudaWaitable::CudaWaitable(std::function<void(void)> on_execute_callback)
    : execute_callback_(on_execute_callback) {
  // create a ros2 guard condition
  guard_condition_ = std::make_shared<rclcpp::GuardCondition>();
}

size_t CudaWaitable::get_number_of_ready_guard_conditions() { return 1; }

void CudaWaitable::add_to_wait_set(rcl_wait_set_t* wait_set) {
  std::lock_guard<std::mutex> lock(guard_condition_mutex_);

  rcl_guard_condition_t* cond = &guard_condition_->get_rcl_guard_condition();
  rcl_ret_t ret = rcl_wait_set_add_guard_condition(wait_set, cond, NULL);

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(
        ret, "failed to add guard condition to wait set");
  }
}

bool CudaWaitable::is_ready(rcl_wait_set_t* wait_set) {
  std::lock_guard<std::mutex> lock(guard_condition_mutex_);

  bool any_ready = false;
  for (size_t ii = 0; ii < wait_set->size_of_guard_conditions; ++ii) {
    const auto* rcl_guard_condition = wait_set->guard_conditions[ii];

    if (nullptr == rcl_guard_condition) {
      continue;
    }
    if (guard_condition_ &&
        &guard_condition_->get_rcl_guard_condition() == rcl_guard_condition) {
      any_ready = true;
      break;
    }
  }
  return any_ready;
}

std::shared_ptr<void> CudaWaitable::take_data() { return nullptr; }

void CudaWaitable::execute(std::shared_ptr<void>& /*data*/) {
  std::lock_guard<std::mutex> lock(execute_mutex_);
  this->execute_callback_();
}

void CudaWaitable::notify() {
  std::lock_guard<std::mutex> lock(notify_guard_conditions_);
  guard_condition_->trigger();
}

// void CudaWaitable::set_goal_handle(
//     std::shared_ptr<AnytimeGoalHandle> goal_handle) {
//   std::lock_guard<std::mutex> lock(mutex);
//   goal_handle_ = goal_handle;
// }

// std::shared_ptr<CudaWaitable::AnytimeGoalHandle>
// CudaWaitable::get_goal_handle() {
//   std::lock_guard<std::mutex> lock(mutex);
//   return goal_handle_;
// }
