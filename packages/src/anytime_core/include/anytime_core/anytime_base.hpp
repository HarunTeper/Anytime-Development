#ifndef ANYTIME_CORE_ANYTIME_BASE_HPP
#define ANYTIME_CORE_ANYTIME_BASE_HPP

#include "anytime_core/anytime_waitable.hpp"
#include "anytime_core/tracing.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <atomic>
#include <memory>

namespace anytime_core
{

template <typename InterfaceType, typename GoalHandleType>
class AnytimeBase
{
public:
  virtual ~AnytimeBase() = default;

  // Domain-specific pure virtual functions that must be implemented by derived classes
  // These handle the actual computation logic
  virtual void compute_single_iteration() = 0;  // Single iteration of computation work
  virtual void populate_feedback(std::shared_ptr<typename InterfaceType::Feedback> feedback) = 0;
  virtual void populate_result(std::shared_ptr<typename InterfaceType::Result> result) = 0;
  virtual void reset_domain_state() = 0;   // Reset domain-specific state
  virtual bool should_finish() const = 0;  // Check if computation should finish

  // Optional: callback to be invoked after each iteration (for async GPU operations)
  // Return nullptr for synchronous operations (default behavior for Monte Carlo)
  // Requirements for get_iteration_callback, sends feedback and calls notify_waitable
  // when batch is complete
  virtual void * get_iteration_callback() { return nullptr; }

  // Optional: override to customize the number of batch iterations
  // Default is batch_size_, but YOLO needs to limit by remaining layers
  virtual int get_batch_iterations() const { return batch_size_; }

  // Combined unified anytime function that handles compute, result, and finish checks
  // This replaces the previous three-function pattern with a single state machine
  virtual void reactive_anytime_function()
  {
    TRACE_REACTIVE_ANYTIME_FUNCTION_ENTRY(node_);
    std::string goal_id_str =
      goal_handle_ ? rclcpp_action::to_string(goal_handle_->get_goal_id()) : "null";
    RCLCPP_DEBUG(
      node_->get_logger(), "[Goal ID: %s] Reactive anytime function called", goal_id_str.c_str());

    if (!is_running()) {
      RCLCPP_DEBUG(
        node_->get_logger(), "[Goal ID: %s] Not running, skipping execution", goal_id_str.c_str());
      return;
    }

    // Step 1: Compute iteration
    compute();

    // Step 3: Check if we should finish
    bool should_finish_now = should_finish();
    bool should_cancel = goal_handle_->is_canceling();

    if ((should_finish_now || should_cancel) && is_running()) {
      // Calculate final result
      calculate_result();

      // Finish the goal
      if (should_cancel) {
        goal_handle_->canceled(result_);
      } else {
        goal_handle_->succeed(result_);
      }

      deactivate();
      RCLCPP_DEBUG(
        node_->get_logger(),
        "[Goal ID: %s] Reactive function finished, should finish: %d, should cancel: %d",
        goal_id_str.c_str(), should_finish_now, should_cancel);
    } else if (is_running()) {
      send_feedback();
      // Continue with next iteration
      notify_waitable();
    }

    TRACE_REACTIVE_ANYTIME_FUNCTION_EXIT(node_, should_finish_now, should_cancel);
  }

  virtual void proactive_anytime_function()
  {
    TRACE_PROACTIVE_ANYTIME_FUNCTION_ENTRY(node_);
    std::string goal_id_str =
      goal_handle_ ? rclcpp_action::to_string(goal_handle_->get_goal_id()) : "null";
    RCLCPP_DEBUG(
      node_->get_logger(), "[Goal ID: %s] Proactive anytime function called", goal_id_str.c_str());

    if (!is_running()) {
      RCLCPP_DEBUG(
        node_->get_logger(), "[Goal ID: %s] Not running, skipping execution", goal_id_str.c_str());
      return;
    }

    // Step 1: Compute iteration
    compute();

    // Step 2: Check if we should finish
    bool should_finish_now = should_finish();
    bool should_cancel = goal_handle_->is_canceling();

    if ((should_finish_now || should_cancel) && is_running()) {
      // Finish the goal
      if (should_cancel) {
        goal_handle_->canceled(result_);
      } else {
        goal_handle_->succeed(result_);
      }

      deactivate();
      RCLCPP_DEBUG(
        node_->get_logger(),
        "[Goal ID: %s] Proactive function finished, should finish: %d, should cancel: %d",
        goal_id_str.c_str(), should_finish_now, should_cancel);
    } else if (is_running()) {
      // Step 3: Calculate and send result/feedback
      calculate_result();
      send_feedback();
      // Continue with next iteration
      notify_waitable();
    }

    TRACE_PROACTIVE_ANYTIME_FUNCTION_EXIT(node_, should_finish_now, should_cancel);
  }

  virtual void start()
  {
    RCLCPP_DEBUG(node_->get_logger(), "Start function called");
    notify_waitable();
  }

  virtual void compute()
  {
    std::string goal_id_str =
      goal_handle_ ? rclcpp_action::to_string(goal_handle_->get_goal_id()) : "null";
    RCLCPP_DEBUG(node_->get_logger(), "[Goal ID: %s] Compute function called", goal_id_str.c_str());

    // Get the number of iterations for this batch
    int iterations = get_batch_iterations();

    TRACE_ANYTIME_COMPUTE_ENTRY(node_, iterations);

    // Start timing
    auto start_time = node_->now();

    // Execute batch iterations
    for (int i = 0; i < iterations; i++) {
      // Check if we should stop processing
      if (goal_handle_->is_canceling() || !goal_handle_->is_executing() || !is_running()) {
        RCLCPP_DEBUG(
          node_->get_logger(), "[Goal ID: %s] Goal handle is canceling, stopping computation",
          goal_id_str.c_str());
        return;
      }

      RCLCPP_DEBUG(
        node_->get_logger(), "[Goal ID: %s] Computing batch iteration %d", goal_id_str.c_str(), i);

      TRACE_ANYTIME_COMPUTE_ITERATION(node_, i);

      // Call domain-specific single iteration computation
      compute_single_iteration();

      RCLCPP_DEBUG(
        node_->get_logger(), "[Goal ID: %s] Finished batch iteration %d", goal_id_str.c_str(), i);
    }

    // End timing
    auto end_time = node_->now();

    // Calculate computation time for this batch
    rclcpp::Duration computation_time = end_time - start_time;

    // Update the average computation time using incremental averaging
    // Using double precision to avoid integer overflow
    batch_count_++;
    if (batch_count_ == 1) {
      average_computation_time_ = computation_time;
    } else {
      // Compute incremental average: avg_new = avg_old + (value - avg_old) / count
      // Use double to avoid overflow with large nanosecond values
      double avg_ns = static_cast<double>(average_computation_time_.nanoseconds());
      double new_ns = static_cast<double>(computation_time.nanoseconds());
      double delta = new_ns - avg_ns;
      avg_ns = avg_ns + (delta / static_cast<double>(batch_count_));

      // Clamp to valid range to prevent overflow
      if (avg_ns < 0.0) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Negative average computation time detected, resetting to current time");
        avg_ns = new_ns;
      }

      average_computation_time_ =
        rclcpp::Duration(std::chrono::nanoseconds(static_cast<int64_t>(avg_ns)));
    }

    RCLCPP_DEBUG(
      node_->get_logger(), "[Goal ID: %s] Average computation time: %f ms", goal_id_str.c_str(),
      average_computation_time_.nanoseconds() / 1e6);

    TRACE_ANYTIME_COMPUTE_EXIT(
      node_, iterations, computation_time.nanoseconds(), average_computation_time_.nanoseconds());
  }

  virtual void send_feedback()
  {
    TRACE_ANYTIME_SEND_FEEDBACK_ENTRY(node_);
    std::string goal_id_str =
      goal_handle_ ? rclcpp_action::to_string(goal_handle_->get_goal_id()) : "null";
    RCLCPP_DEBUG(
      node_->get_logger(), "[Goal ID: %s] Send feedback function called", goal_id_str.c_str());
    auto feedback = std::make_shared<typename InterfaceType::Feedback>();

    // Let derived class populate domain-specific feedback
    populate_feedback(feedback);

    if (goal_handle_) {
      goal_handle_->publish_feedback(feedback);
      RCLCPP_DEBUG(node_->get_logger(), "[Goal ID: %s] Feedback sent", goal_id_str.c_str());
    } else {
      RCLCPP_WARN(node_->get_logger(), "Goal handle is null, cannot send feedback");
    }

    TRACE_ANYTIME_SEND_FEEDBACK_EXIT(node_);
  }

  virtual void calculate_result()
  {
    TRACE_ANYTIME_CALCULATE_RESULT_ENTRY(node_);
    std::string goal_id_str =
      goal_handle_ ? rclcpp_action::to_string(goal_handle_->get_goal_id()) : "null";
    RCLCPP_DEBUG(
      node_->get_logger(), "[Goal ID: %s] Calculate result function called", goal_id_str.c_str());
    auto new_result = std::make_shared<typename InterfaceType::Result>();

    // Let derived class populate domain-specific result
    populate_result(new_result);

    result_ = new_result;

    TRACE_ANYTIME_CALCULATE_RESULT_EXIT(node_);
  }

  virtual void notify_cancel()
  {
    std::string goal_id_str =
      goal_handle_ ? rclcpp_action::to_string(goal_handle_->get_goal_id()) : "null";
    RCLCPP_DEBUG(node_->get_logger(), "[Goal ID: %s] Notify cancel function", goal_id_str.c_str());
    // Trigger the waitable to process the cancellation
    // this->notify_waitable();
    RCLCPP_DEBUG(
      node_->get_logger(), "[Goal ID: %s] Notify cancel function finished", goal_id_str.c_str());
  }

  virtual void reset()
  {
    TRACE_ANYTIME_BASE_RESET(node_);
    std::string goal_id_str =
      goal_handle_ ? rclcpp_action::to_string(goal_handle_->get_goal_id()) : "null";
    RCLCPP_DEBUG(node_->get_logger(), "[Goal ID: %s] Reset function called", goal_id_str.c_str());
    result_ = std::make_shared<typename InterfaceType::Result>();

    // Reset domain-specific state
    reset_domain_state();

    batch_count_ = 0;
    average_computation_time_ = rclcpp::Duration(0, 0);
  }

  // Activation control
  void activate()
  {
    TRACE_ANYTIME_BASE_ACTIVATE(node_);
    is_running_.store(true, std::memory_order_seq_cst);
  }
  void deactivate()
  {
    TRACE_ANYTIME_BASE_DEACTIVATE(node_);
    is_running_.store(false, std::memory_order_seq_cst);
  }
  bool is_running() const { return is_running_.load(std::memory_order_seq_cst); }

  // Goal handle management
  void set_goal_handle(std::shared_ptr<GoalHandleType> goal_handle) { goal_handle_ = goal_handle; }
  std::shared_ptr<GoalHandleType> get_goal_handle() { return goal_handle_; }

  // Waitable notification - single unified waitable
  void notify_waitable()
  {
    RCLCPP_DEBUG(node_->get_logger(), "Notifying waitable");
    anytime_waitable_->trigger();
  }

protected:
  // Initialize waitables and callback group
  // Call this from derived class constructor with is_reactive_proactive template parameter
  template <bool isReactiveProactive>
  void initialize_anytime_base(rclcpp::Node * node, int batch_size)
  {
    node_ = node;
    batch_size_ = batch_size;

    // Store the mode
    is_reactive_proactive_ = isReactiveProactive;

    TRACE_ANYTIME_BASE_INIT(node_, batch_size_, isReactiveProactive);

    // Create callback group
    compute_callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Create single unified waitable based on reactive/proactive mode
    if constexpr (isReactiveProactive) {
      // Proactive mode
      anytime_waitable_ =
        std::make_shared<AnytimeWaitable>([this]() { this->proactive_anytime_function(); });
    } else {
      // Reactive mode
      anytime_waitable_ =
        std::make_shared<AnytimeWaitable>([this]() { this->reactive_anytime_function(); });
    }

    // Add waitable to node
    node->get_node_waitables_interface()->add_waitable(anytime_waitable_, compute_callback_group_);
  }

  // Node pointer for logging (must be set by derived class)
  rclcpp::Node * node_ = nullptr;

  // Single unified waitable for all operations
  std::shared_ptr<AnytimeWaitable> anytime_waitable_;

  // Running state
  std::atomic<bool> is_running_{false};

  // Goal handle
  std::shared_ptr<GoalHandleType> goal_handle_;

  // Feedback and result messages
  std::shared_ptr<typename InterfaceType::Feedback> feedback_ =
    std::make_shared<typename InterfaceType::Feedback>();

  std::shared_ptr<typename InterfaceType::Result> result_ =
    std::make_shared<typename InterfaceType::Result>();

  // Callback group for compute operations
  rclcpp::CallbackGroup::SharedPtr compute_callback_group_;

  // Batch size configuration
  int batch_size_ = 1;

  // Performance metrics
  uint64_t batch_count_ = 0;
  rclcpp::Duration average_computation_time_{0, 0};

  // Mode tracking
  bool is_reactive_proactive_ = false;
};

}  // namespace anytime_core

#endif  // ANYTIME_CORE_ANYTIME_BASE_HPP