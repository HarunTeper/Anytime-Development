#ifndef MONTE_CARLO_HPP
#define MONTE_CARLO_HPP

#include <cstdint>
#include <future>
#include <random>
#include "anytime_monte_carlo/anytime_base.hpp"

// Aliases for better readability
using Anytime = anytime_interfaces::action::Anytime;
using AnytimeGoalHandle = rclcpp_action::ServerGoalHandle<Anytime>;

// Monte Carlo Pi class template
template <bool isReactive, bool separate_thread, bool multi_threading>
class MonteCarloPi : public AnytimeBase<double, Anytime, AnytimeGoalHandle> {
 public:
  // Constructor
  MonteCarloPi(rclcpp::Node* node, int batch_size = 1)
      : node_(node), batch_size_(batch_size) {
    // --- Reactive Variables ---
    if constexpr (isReactive && multi_threading) {
      anytime_iteration_waitable_ = std::make_shared<AnytimeWaitable>(
          [this]() { this->reactive_function(); });
    } else if constexpr (isReactive && !multi_threading) {
      anytime_iteration_waitable_ = std::make_shared<AnytimeWaitable>(
          [this]() { this->reactive_function(); });
    }

    // --- Proactive Variables ---
    // Iteration Waitables
    // Iteration for single-threaded, loop for multi-threaded
    if constexpr (!isReactive && multi_threading) {
      anytime_iteration_waitable_ = std::make_shared<AnytimeWaitable>(
          [this]() { this->proactive_function(); });
    } else if constexpr (!isReactive && !multi_threading) {
      anytime_iteration_waitable_ = std::make_shared<AnytimeWaitable>(
          [this]() { this->proactive_function(); });
    }

    if constexpr (!isReactive) {
      anytime_check_finish_waitable_ = std::make_shared<AnytimeWaitable>(
          [this]() { this->check_cancel_and_finish_proactive(); });
    }

    // callback group
    compute_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    if constexpr (!separate_thread) {
      if constexpr (isReactive) {
        // add the waitable to the node
        node_->get_node_waitables_interface()->add_waitable(
            anytime_iteration_waitable_, compute_callback_group_);
      } else if constexpr (!isReactive) {
        // add the waitable to the node
        node_->get_node_waitables_interface()->add_waitable(
            anytime_iteration_waitable_, compute_callback_group_);

        node_->get_node_waitables_interface()->add_waitable(
            anytime_check_finish_waitable_,
            node_->get_node_base_interface()->get_default_callback_group());
      }
    } else if constexpr (separate_thread) {
      if constexpr (isReactive) {
        // nothing to do
      } else if constexpr (!isReactive) {
        node_->get_node_waitables_interface()->add_waitable(
            anytime_check_finish_waitable_,
            node_->get_node_base_interface()->get_default_callback_group());
      }
    }
  }

  // ----------------- Reactive Functions -----------------

  // Blocking function to approximate Pi
  void reactive_function() {
    if (check_cancel_reactive() || check_finish_reactive()) {
      return;
    } else {
      compute();
      notify_iteration();
    }
  }

  // Blocking function to approximate Pi
  void reactive_function_loop() {
    while (true) {
      if (check_cancel_reactive() || check_finish_reactive()) {
        return;
      } else {
        compute();
      }
    }
  }

  bool check_cancel_reactive() {
    if (this->goal_handle_->is_canceling()) {
      RCLCPP_INFO(node_->get_logger(), "Canceling goal");
      this->calculate_result();
      this->result_->action_end = this->node_->now();
      this->goal_handle_->canceled(this->result_);
      this->deactivate();
      return true;
    }
    return false;
  }

  bool check_finish_reactive() {
    if (loop_count_ >= this->goal_handle_->get_goal()->goal) {
      RCLCPP_INFO(node_->get_logger(), "Finishing Goal");
      this->calculate_result();
      this->result_->action_end = this->node_->now();
      this->goal_handle_->succeed(this->result_);
      this->deactivate();
      return true;
    }
    return false;
  }

  // ----------------- Proactive Functions -----------------

  // proactive function to approximate Pi
  void proactive_function() {
    compute();
    calculate_result();
    notify_check_finish();
  }

  void proactive_function_loop() {
    while (true) {
      if (check_cancel_proactive() || check_finish_proactive()) {
        notify_check_finish();
        return;
      } else {
        compute();
        calculate_result();
      }
    }
  }

  void cancel_proactive() {
    if (this->goal_handle_->is_canceling()) {
      this->result_->action_end = this->node_->now();
      this->goal_handle_->canceled(this->result_);
      this->deactivate();
    }
  }

  void finish_proactive() {
    this->result_->action_end = this->node_->now();
    this->goal_handle_->succeed(this->result_);
    this->deactivate();
  }

  bool check_cancel_proactive() {
    if (this->goal_handle_->is_canceling() ||
        !this->goal_handle_->is_active() ||
        !this->goal_handle_->is_executing()) {
      return true;
    }
    return false;
  }

  bool check_finish_proactive() {
    // print loop count and goal handle goal
    if (loop_count_ >= this->goal_handle_->get_goal()->goal) {
      return true;
    }
    return false;
  }

  void check_cancel_and_finish_proactive() {
    if (check_cancel_proactive()) {
      cancel_proactive();
      return;
    } else if (check_finish_proactive()) {
      finish_proactive();
    } else
      notify_iteration();
  }

  // ----------------- Common Functions -----------------

  void compute() {
    for (int i = 0; i < batch_size_; ++i) {
      x = (float)rand() / RAND_MAX;
      y = (float)rand() / RAND_MAX;

      if (sqrt(pow(x, 2) + pow(y, 2)) <= 1) {
        count_inside_++;
      }
      count_total_++;

      loop_count_++;
    }
  }

  void calculate_result() override {
    // Calculate the result
    this->result_->action_compute_end = this->node_->now();
    this->result_->result = 4 * (double)count_inside_ / count_total_;
    this->result_->iterations = loop_count_;
  }

  // Cancel function
  void cancel() override {
    this->result_->action_cancel = this->node_->now();
    if constexpr (isReactive) {
      // nothing to do
    } else if constexpr (!isReactive) {
      notify_check_finish();
    }
  }

  // Reset function
  void reset() override {
    // Reset the count variables
    count_total_ = 0;
    count_inside_ = 0;
    count_outside_ = 0;

    loop_count_ = 0;

    finished_ = false;
    canceled_ = false;

    this->result_->client_start = this->goal_handle_->get_goal()->client_start;
    this->result_->action_send = this->goal_handle_->get_goal()->action_send;
    this->result_->action_accept = this->goal_handle_accept_time_;
  }

  void start() override {
    this->result_->action_start = node_->now();
    if constexpr (separate_thread) {
      std::thread([this]() {
        if constexpr (isReactive) {
          this->reactive_function_loop();
        } else if constexpr (!isReactive) {
          this->proactive_function_loop();
        }
      }).detach();
    } else {
      notify_iteration();
    }
  }

 protected:
  rclcpp::Node* node_;  // Node reference for logging
  int batch_size_;      // Batch size for compute iterations

  // Count variables
  int count_total_ = 0;
  int count_inside_ = 0;
  int count_outside_ = 0;

  int loop_count_ = 0;

  float x = 0.0;
  float y = 0.0;
};

#endif  // MONTE_CARLO_HPP