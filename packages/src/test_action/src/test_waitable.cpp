#include "test_action/test_waitable.hpp"

#include "rclcpp/exceptions.hpp"

#include <iostream>

namespace test_action
{

TestWaitable::TestWaitable(std::function<void(void)> on_execute_callback)
: execute_callback_(on_execute_callback)
{
  // Create a shared guard condition
  guard_condition_ = std::make_shared<rclcpp::GuardCondition>();
}

void TestWaitable::add_to_wait_set(rcl_wait_set_t * wait_set)
{
  std::lock_guard<std::mutex> lock(guard_condition_mutex_);

  if (guard_condition_) {
    auto rcl_guard_condition = &guard_condition_->get_rcl_guard_condition();

    rcl_ret_t ret = rcl_wait_set_add_guard_condition(wait_set, rcl_guard_condition, NULL);

    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to add guard condition to wait set");
    }
  }
}

bool TestWaitable::is_ready(rcl_wait_set_t * wait_set)
{
  std::lock_guard<std::mutex> lock(guard_condition_mutex_);

  if (!guard_condition_) {
    return false;
  }

  auto rcl_guard_condition = &guard_condition_->get_rcl_guard_condition();

  for (size_t ii = 0; ii < wait_set->size_of_guard_conditions; ++ii) {
    if (wait_set->guard_conditions[ii] == rcl_guard_condition) {
      return true;
    }
  }

  return false;
}

void TestWaitable::execute(std::shared_ptr<void> & data)
{
  (void)data;
  if (execute_callback_) {
    execute_callback_();
  }
}

std::shared_ptr<void> TestWaitable::take_data() { return nullptr; }

std::shared_ptr<void> TestWaitable::take_data_by_entity_id(size_t id)
{
  (void)id;
  return nullptr;
}

void TestWaitable::set_on_ready_callback(std::function<void(size_t, int)> callback)
{
  auto gc_callback = [callback](size_t count) { callback(count, 0); };

  std::lock_guard<std::mutex> lock(guard_condition_mutex_);

  on_ready_callback_ = gc_callback;
  if (guard_condition_) {
    guard_condition_->set_on_trigger_callback(on_ready_callback_);
  }
}

void TestWaitable::clear_on_ready_callback()
{
  std::lock_guard<std::mutex> lock(guard_condition_mutex_);

  on_ready_callback_ = nullptr;
  if (guard_condition_) {
    guard_condition_->set_on_trigger_callback(nullptr);
  }
}

size_t TestWaitable::get_number_of_ready_guard_conditions()
{
  std::lock_guard<std::mutex> lock(guard_condition_mutex_);
  return guard_condition_ ? 1 : 0;
}

void TestWaitable::trigger()
{
  std::lock_guard<std::mutex> lock(guard_condition_mutex_);
  if (guard_condition_) {
    guard_condition_->trigger();
  }
}

rclcpp::GuardCondition & TestWaitable::get_guard_condition() { return *guard_condition_; }

}  // namespace test_action
