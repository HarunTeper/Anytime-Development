#ifndef ANYTIME_CORE_ANYTIME_WAITABLE_HPP
#define ANYTIME_CORE_ANYTIME_WAITABLE_HPP

#include "rclcpp/exceptions.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/waitable.hpp"

#include <functional>
#include <memory>
#include <mutex>

namespace anytime_core
{

/// Custom waitable that uses a guard condition to trigger execution
class AnytimeWaitable : public rclcpp::Waitable
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(AnytimeWaitable)

  // Constructor
  explicit AnytimeWaitable(std::function<void(void)> on_execute_callback = {})
  : execute_callback_(on_execute_callback)
  {
    // Create a shared guard condition
    guard_condition_ = std::make_shared<rclcpp::GuardCondition>();
  }

  // Destructor
  ~AnytimeWaitable() override = default;

  /// Add conditions to the wait set
  void add_to_wait_set(rcl_wait_set_t * wait_set) override
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

  /// Check conditions against the wait set
  bool is_ready(rcl_wait_set_t * wait_set) override
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

  /// Perform work associated with the waitable.
  void execute(std::shared_ptr<void> & data) override
  {
    (void)data;
    if (execute_callback_) {
      execute_callback_();
    }
  }

  /// Retrieve data to be used in the next execute call.
  std::shared_ptr<void> take_data() override { return nullptr; }

  /// Take the data from an entity ID so that it can be consumed with `execute`.
  std::shared_ptr<void> take_data_by_entity_id(size_t id) override
  {
    (void)id;
    return nullptr;
  }

  /// Set a callback to be called whenever the waitable becomes ready.
  void set_on_ready_callback(std::function<void(size_t, int)> callback) override
  {
    auto gc_callback = [callback](size_t count) { callback(count, 0); };

    std::lock_guard<std::mutex> lock(guard_condition_mutex_);

    on_ready_callback_ = gc_callback;
    if (guard_condition_) {
      guard_condition_->set_on_trigger_callback(on_ready_callback_);
    }
  }

  /// Unset any callback registered via set_on_ready_callback.
  void clear_on_ready_callback() override
  {
    std::lock_guard<std::mutex> lock(guard_condition_mutex_);

    on_ready_callback_ = nullptr;
    if (guard_condition_) {
      guard_condition_->set_on_trigger_callback(nullptr);
    }
  }

  /// Get the number of ready guard_conditions
  size_t get_number_of_ready_guard_conditions() override
  {
    std::lock_guard<std::mutex> lock(guard_condition_mutex_);
    return guard_condition_ ? 1 : 0;
  }

  /// Trigger the guard condition
  void trigger()
  {
    std::lock_guard<std::mutex> lock(guard_condition_mutex_);
    if (guard_condition_) {
      guard_condition_->trigger();
    }
  }

  /// Get the guard condition
  rclcpp::GuardCondition & get_guard_condition() { return *guard_condition_; }

private:
  /// Callback to run when waitable executes
  std::function<void(void)> execute_callback_;

  std::mutex guard_condition_mutex_;

  std::function<void(size_t)> on_ready_callback_;

  /// The guard condition to be waited on
  std::shared_ptr<rclcpp::GuardCondition> guard_condition_;
};

}  // namespace anytime_core

#endif  // ANYTIME_CORE_ANYTIME_WAITABLE_HPP
