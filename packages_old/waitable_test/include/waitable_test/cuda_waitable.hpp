#include <rcl/event.h>
#include <rcl/wait.h>
#include <rclcpp/guard_condition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/waitable.hpp>
#include "anytime_interfaces/action/anytime.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class CudaWaitable : public rclcpp::Waitable {
 public:
  using Anytime = anytime_interfaces::action::Anytime;
  using AnytimeGoalHandle = rclcpp_action::ServerGoalHandle<Anytime>;

  CudaWaitable(std::function<void(void)> on_execute_callback);

  RCLCPP_PUBLIC
  size_t get_number_of_ready_guard_conditions() override;

  // execute
  void execute(std::shared_ptr<void>& data) override;

  // take_data
  std::shared_ptr<void> take_data() override;

  // is ready
  bool is_ready(rcl_wait_set_t* wait_set) override;

  // add to wait set
  void add_to_wait_set(rcl_wait_set_t* wait_set) override;

  // create a ros2 guard condition
  std::shared_ptr<rclcpp::GuardCondition> guard_condition_;

  // mutex
  std::mutex mutex;
  std::mutex guard_condition_mutex_;
  std::mutex notify_guard_conditions_;
  std::mutex execute_mutex_;

  // rclcpp anytime goal handle
  std::shared_ptr<AnytimeGoalHandle> goal_handle_;

  // set and get for goal_handle
  void set_goal_handle(std::shared_ptr<AnytimeGoalHandle> goal_handle);
  std::shared_ptr<AnytimeGoalHandle> get_goal_handle();

  std::function<void(void)> execute_callback_;

  void notify();

 private:
};