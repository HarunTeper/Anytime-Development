#include "interference/interference_timer_node.hpp"

#include "anytime_tracing/anytime_tracetools.h"
#include "rclcpp_components/register_node_macro.hpp"

#include <chrono>

namespace interference
{

InterferenceTimerNode::InterferenceTimerNode(const rclcpp::NodeOptions & options)
: Node("interference_timer", options), execution_count_(0)
{
  // Declare and get parameters
  this->declare_parameter<int>("timer_period_ms", 100);
  this->declare_parameter<int>("execution_time_ms", 10);

  this->get_parameter("timer_period_ms", timer_period_ms_);
  this->get_parameter("execution_time_ms", execution_time_ms_);

  // Add custom tracepoint for interference timer initialization
  ANYTIME_TRACEPOINT(
    interference_timer_init, static_cast<const void *>(this->get_node_base_interface().get()),
    timer_period_ms_, execution_time_ms_);

  RCLCPP_INFO(
    this->get_logger(),
    "Interference Timer Node initialized with period=%d ms, execution_time=%d ms", timer_period_ms_,
    execution_time_ms_);

  // Create timer
  auto timer_period = std::chrono::milliseconds(timer_period_ms_);
  timer_ =
    this->create_wall_timer(timer_period, std::bind(&InterferenceTimerNode::timer_callback, this));
}

InterferenceTimerNode::~InterferenceTimerNode()
{
  RCLCPP_INFO(
    this->get_logger(), "Interference Timer Node shutting down. Total executions: %zu",
    execution_count_);
}

void InterferenceTimerNode::timer_callback()
{
  ANYTIME_TRACEPOINT(
    interference_timer_callback_entry,
    static_cast<const void *>(this->get_node_base_interface().get()), execution_count_);

  auto start_time = std::chrono::steady_clock::now();

  RCLCPP_DEBUG(this->get_logger(), "Timer callback started (execution #%zu)", execution_count_);

  // Busy-wait loop to simulate interference
  auto target_duration = std::chrono::milliseconds(execution_time_ms_);
  while (std::chrono::steady_clock::now() - start_time < target_duration) {
    // Busy-wait (spin loop)
    // This intentionally blocks to create interference
  }

  auto end_time = std::chrono::steady_clock::now();
  auto actual_duration_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();

  execution_count_++;

  RCLCPP_DEBUG(
    this->get_logger(), "Timer callback completed (execution #%zu). Duration: %ld ns",
    execution_count_, actual_duration_ns);

  ANYTIME_TRACEPOINT(
    interference_timer_callback_exit,
    static_cast<const void *>(this->get_node_base_interface().get()), execution_count_,
    actual_duration_ns);
}

}  // namespace interference

// Register the component
RCLCPP_COMPONENTS_REGISTER_NODE(interference::InterferenceTimerNode)
