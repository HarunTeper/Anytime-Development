#ifndef ANYTIME_WAITABLE_HPP
#define ANYTIME_WAITABLE_HPP

#include <rclcpp/guard_condition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/waitable.hpp>

#include <rcl/event.h>
#include <rcl/wait.h>

class AnytimeWaitable : public rclcpp::Waitable
{
public:
  AnytimeWaitable(std::function<void(void)> on_execute_callback)
  : execute_callback_(on_execute_callback)
  {
    // create a ros2 guard condition
    guard_condition_ = std::make_shared<rclcpp::GuardCondition>();
  }

  RCLCPP_PUBLIC
  size_t get_number_of_ready_guard_conditions() override { return 1; }

  void execute(std::shared_ptr<void> & data) override
  {
    (void)data;
    std::lock_guard<std::mutex> lock(execute_mutex_);
    this->execute_callback_();
  }

  std::shared_ptr<void> take_data() override { return nullptr; }

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

  void add_to_wait_set(rcl_wait_set_t * wait_set) override
  {
    std::lock_guard<std::mutex> lock(guard_condition_mutex_);

    rcl_guard_condition_t * cond = &guard_condition_->get_rcl_guard_condition();
    rcl_ret_t ret = rcl_wait_set_add_guard_condition(wait_set, cond, NULL);

    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to add guard condition to wait set");
    }
  }

  void notify()
  {
    std::lock_guard<std::mutex> lock(notify_guard_conditions_);
    guard_condition_->trigger();
  }

  std::shared_ptr<rclcpp::GuardCondition> guard_condition_;

private:
  std::atomic<bool> triggered_{false};

  std::mutex guard_condition_mutex_;
  std::mutex notify_guard_conditions_;
  std::mutex execute_mutex_;

  std::function<void(void)> execute_callback_;
};

#endif  // ANYTIME_WAITABLE_HPP