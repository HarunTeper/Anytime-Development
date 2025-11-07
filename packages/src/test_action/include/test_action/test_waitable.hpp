#ifndef TEST_ACTION__TEST_WAITABLE_HPP_
#define TEST_ACTION__TEST_WAITABLE_HPP_

#include "rclcpp/guard_condition.hpp"
#include "rclcpp/waitable.hpp"

#include <functional>
#include <memory>
#include <mutex>

namespace test_action
{

/// Custom waitable that uses a guard condition to trigger execution
class TestWaitable : public rclcpp::Waitable
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(TestWaitable)

  // Constructor
  explicit TestWaitable(std::function<void(void)> on_execute_callback = {});

  // Destructor
  ~TestWaitable() override = default;

  /// Add conditions to the wait set
  void add_to_wait_set(rcl_wait_set_t * wait_set) override;

  /// Check conditions against the wait set
  bool is_ready(rcl_wait_set_t * wait_set) override;

  /// Perform work associated with the waitable.
  void execute(std::shared_ptr<void> & data) override;

  /// Retrieve data to be used in the next execute call.
  std::shared_ptr<void> take_data() override;

  /// Take the data from an entity ID so that it can be consumed with `execute`.
  std::shared_ptr<void> take_data_by_entity_id(size_t id) override;

  /// Set a callback to be called whenever the waitable becomes ready.
  void set_on_ready_callback(std::function<void(size_t, int)> callback) override;

  /// Unset any callback registered via set_on_ready_callback.
  void clear_on_ready_callback() override;

  /// Get the number of ready guard_conditions
  size_t get_number_of_ready_guard_conditions() override;

  /// Trigger the guard condition
  void trigger();

  /// Get the guard condition
  rclcpp::GuardCondition & get_guard_condition();

private:
  /// Callback to run when waitable executes
  std::function<void(void)> execute_callback_;

  std::mutex guard_condition_mutex_;

  std::function<void(size_t)> on_ready_callback_;

  /// The guard condition to be waited on
  std::shared_ptr<rclcpp::GuardCondition> guard_condition_;
};

}  // namespace test_action

#endif  // TEST_ACTION__TEST_WAITABLE_HPP_
