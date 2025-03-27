#ifndef MONTE_CARLO_HPP
#define MONTE_CARLO_HPP

#include "anytime_monte_carlo/anytime_base.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <cstdint>
#include <future>
#include <random>

// Aliases for better readability
using Anytime = anytime_interfaces::action::MonteCarlo;
using AnytimeGoalHandle = rclcpp_action::ServerGoalHandle<Anytime>;

// Monte Carlo Pi class template
template <bool isReactiveProactive, bool isSingleMulti>
class MonteCarloPi : public AnytimeBase<double, Anytime, AnytimeGoalHandle>
{
public:
  // Constructor
  MonteCarloPi(rclcpp::Node * node, int batch_size = 1) : node_(node), batch_size_(batch_size)
  {
    // callback group
    compute_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // --- Proactive Variables ---
    if constexpr (isReactiveProactive) {
      anytime_iteration_waitable_ =
        std::make_shared<AnytimeWaitable>([this]() { this->proactive_function(); });
      anytime_check_finish_waitable_ =
        std::make_shared<AnytimeWaitable>([this]() { this->check_cancel_and_finish_proactive(); });
      node_->get_node_waitables_interface()->add_waitable(
        anytime_iteration_waitable_, compute_callback_group_);
      node_->get_node_waitables_interface()->add_waitable(
        anytime_check_finish_waitable_,
        node_->get_node_base_interface()->get_default_callback_group());
    }
    // --- Reactive Variables ---
    else {
      anytime_iteration_waitable_ =
        std::make_shared<AnytimeWaitable>([this]() { this->reactive_function(); });
      node_->get_node_waitables_interface()->add_waitable(
        anytime_iteration_waitable_, compute_callback_group_);
    }
  }

  // ----------------- Reactive Functions -----------------

  void reactive_function()
  {
    if (check_cancel_and_finish_reactive()) {
      return;
    } else {
      compute();
      notify_iteration();
    }
  }

  bool check_cancel_and_finish_reactive() override
  {
    bool should_finish = loop_count_ >= this->goal_handle_->get_goal()->goal;
    bool should_cancel = this->goal_handle_->is_canceling();

    if (should_finish || should_cancel) {
      this->result_->action_server_send_result = this->node_->now();
      this->result_->batch_time = this->average_computation_time_;
      this->calculate_result();

      if (should_cancel) {
        this->goal_handle_->canceled(this->result_);
      } else {
        this->goal_handle_->succeed(this->result_);
      }

      this->deactivate();
      return true;
    }

    return false;
  }

  // ----------------- Proactive Functions -----------------

  // proactive function to approximate Pi
  void proactive_function()
  {
    compute();
    calculate_result();
    notify_check_finish();
  }

  void check_cancel_and_finish_proactive() override
  {
    bool should_finish = loop_count_ >= this->goal_handle_->get_goal()->goal;
    bool should_cancel = this->goal_handle_->is_canceling() || !this->goal_handle_->is_executing();

    if ((should_finish || should_cancel) && this->is_running()) {
      this->result_->action_server_send_result = this->node_->now();

      this->result_->batch_time = average_computation_time_;

      if (should_cancel) {
        this->goal_handle_->canceled(this->result_);
      } else {
        this->goal_handle_->succeed(this->result_);
      }

      this->deactivate();
      return;
    } else if (!this->is_running()) {
      return;
    }

    notify_iteration();
  }

  // ----------------- Common Functions -----------------

  void start() override { notify_iteration(); }

  void compute() override
  {
    // Start timing
    auto start_time = this->node_->now();

    for (int i = 0; i < batch_size_; i++) {
      x = (float)rand() / RAND_MAX;
      y = (float)rand() / RAND_MAX;

      if (sqrt(pow(x, 2) + pow(y, 2)) <= 1) {
        count_inside_++;
      }
      count_total_++;

      loop_count_++;
    }

    // End timing
    auto end_time = this->node_->now();
    // Calculate computation time for this batch
    rclcpp::Duration computation_time = end_time - start_time;

    // Update the average computation time
    batch_count_++;
    if (batch_count_ == 1) {
      average_computation_time_ = computation_time;
    } else {
      average_computation_time_ = rclcpp::Duration(std::chrono::nanoseconds(
        average_computation_time_.nanoseconds() +
        (computation_time.nanoseconds() - average_computation_time_.nanoseconds()) / batch_count_));
    }
  }

  void calculate_result() override
  {
    // Calculate the result
    this->result_->result = 4 * (double)count_inside_ / count_total_;
    this->result_->iterations = loop_count_;

    // Add additional information to result
    this->result_->batch_size = batch_size_;
  }

  // Cancel function
  void notify_cancel() override
  {
    this->result_->action_server_cancel = this->node_->now();
    if constexpr (isReactiveProactive) {
      notify_check_finish();
    } else if constexpr (!isReactiveProactive) {
      // nothing to do
    }
  }

  // Reset function
  void reset() override
  {
    // Reset the count variables
    count_total_ = 0;
    count_inside_ = 0;
    count_outside_ = 0;

    loop_count_ = 0;

    this->result_->action_server_receive = this->server_goal_receive_time_;
    this->result_->action_server_accept = this->server_goal_accept_time_;
    this->result_->action_server_start = this->server_goal_start_time_;

    batch_count_ = 0;
    average_computation_time_ = rclcpp::Duration(0, 0);
  }

protected:
  rclcpp::Node * node_;  // Node reference for logging
  int batch_size_;       // Batch size for compute iterations

  // Count variables
  int count_total_ = 0;
  int count_inside_ = 0;
  int count_outside_ = 0;

  int loop_count_ = 0;

  float x = 0.0;
  float y = 0.0;

  // Batch count and average computation time
  int batch_count_ = 0;
  rclcpp::Duration average_computation_time_{0, 0};  // in milliseconds
};

#endif  // MONTE_CARLO_HPP