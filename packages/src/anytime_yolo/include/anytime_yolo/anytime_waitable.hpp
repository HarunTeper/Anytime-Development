#ifndef ANYTIME_WAITABLE_HPP
#define ANYTIME_WAITABLE_HPP

#include <rclcpp/guard_condition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/waitable.hpp>

#include <rcl/event.h>
#include <rcl/wait.h>

#include <atomic>  // Include atomic header for thread-safe operations

class AnytimeWaitable : public rclcpp::Waitable
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(AnytimeWaitable)

  AnytimeWaitable(std::function<void(void)> on_execute_callback)
  : execute_callback_(on_execute_callback)
  {
    // create a ros2 guard condition
    guard_condition_ = std::make_shared<rclcpp::GuardCondition>();
  }

  ~AnytimeWaitable() override = default;

  void add_to_wait_set(rcl_wait_set_t * wait_set) override
  {
    std::lock_guard<std::mutex> lock(guard_condition_mutex_);

    rcl_guard_condition_t * cond = &guard_condition_->get_rcl_guard_condition();
    rcl_ret_t ret = rcl_wait_set_add_guard_condition(wait_set, cond, NULL);

    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to add guard condition to wait set");
    }
  }

  bool is_ready(rcl_wait_set_t * wait_set) override
  {
    std::lock_guard<std::mutex> lock(guard_condition_mutex_);

    bool any_ready = false;
    for (size_t ii = 0; ii < wait_set->size_of_guard_conditions; ++ii) {
      const auto * rcl_guard_condition = wait_set->guard_conditions[ii];

      if (nullptr == rcl_guard_condition) {
        continue;
      }
      if (guard_condition_ && &guard_condition_->get_rcl_guard_condition() == rcl_guard_condition) {
        any_ready = true;
        break;
      }
    }
    return any_ready;
  }

  void execute(std::shared_ptr<void> & data) override
  {
    (void)data;
    this->execute_callback_();
    is_triggered_ = false;  // Reset the flag atomically after execution
  }

  std::shared_ptr<void> take_data() override { return nullptr; }

  RCLCPP_PUBLIC
  size_t get_number_of_ready_guard_conditions() override { return 1; }

  void notify()
  {
    std::lock_guard<std::mutex> lock(guard_condition_mutex_);
    if (!is_triggered_.exchange(true)) {  // Atomically check and set the flag
      guard_condition_->trigger();
    }
  }

private:
  std::shared_ptr<rclcpp::GuardCondition> guard_condition_;
  std::mutex guard_condition_mutex_;
  std::mutex notify_guard_conditions_;
  std::function<void(void)> execute_callback_;
  std::atomic<bool> is_triggered_ = false;  // Use atomic for thread-safe flag
};

#endif  // ANYTIME_WAITABLE_HPP