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
template <bool isActive, bool separate_thread>
class MonteCarloPi : public AnytimeBase<double, Anytime, AnytimeGoalHandle> {
 public:
  // Constructor
  MonteCarloPi(rclcpp::Node* node) : node_(node) {
    anytime_waitable_ = std::make_shared<AnytimeWaitable>([this]() {
      if constexpr (isActive) {
        this->active_function(this->goal_handle_);
      } else {
        this->passive_function(this->goal_handle_);
      }
    });

    anytime_finish_waitable_ = std::make_shared<AnytimeWaitable>(
        [this]() { this->check_cancel_and_finish(); });

    // add the waitable to the node
    node_->get_node_waitables_interface()->add_waitable(
        this->get_anytime_waitable(),
        node_->get_node_base_interface()->get_default_callback_group());

    node_->get_node_waitables_interface()->add_waitable(
        this->get_anytime_finish_waitable(),
        node_->get_node_base_interface()->get_default_callback_group());
  }

  // Blocking function to approximate Pi
  void active_function(const std::shared_ptr<AnytimeGoalHandle> goal_handle) {
    if (check_cancel_and_finish()) {
      return;
    }
    if (loop_count_ < goal_handle->get_goal()->goal) {
      x = (float)rand() / RAND_MAX;
      y = (float)rand() / RAND_MAX;

      if (sqrt(pow(x, 2) + pow(y, 2)) <= 1) {
        count_inside_++;
      }
      count_total_++;

      loop_count_++;

      this->anytime_waitable_->notify();
    } else {
      RCLCPP_INFO(node_->get_logger(), "Goal was completed");
      check_cancel_and_finish();
    }
  }

  // Passive function to approximate Pi
  void passive_function(const std::shared_ptr<AnytimeGoalHandle> goal_handle) {
    if (loop_count_ < goal_handle->get_goal()->goal) {
      x = (float)rand() / RAND_MAX;
      y = (float)rand() / RAND_MAX;

      if (sqrt(pow(x, 2) + pow(y, 2)) <= 1) {
        count_inside_++;
      }
      count_total_++;

      loop_count_++;

      calculate_result();
      this->anytime_waitable_->notify();
    }
  }

  // Cancel function
  void cancel() override {
    if constexpr (!isActive) {
      this->notify_finish();
    }
  }

  // Reset function
  void reset() override {
    // Reset the count variables
    count_total_ = 0;
    count_inside_ = 0;
    count_outside_ = 0;

    loop_count_ = 0;

    this->result_->client_start = this->goal_handle_->get_goal()->client_start;
    this->result_->action_send = this->goal_handle_->get_goal()->action_send;
    this->result_->action_accept = this->goal_handle_accept_time_;
    this->result_->action_start = node_->now();
  }

  void calculate_result() override {
    // Calculate the result
    this->result_->result = 4 * (double)count_inside_ / count_total_;
    this->result_->iterations = loop_count_;
  }

  bool check_cancel_and_finish() override {
    if (this->goal_handle_->is_canceling()) {
      RCLCPP_INFO(node_->get_logger(), "Canceling goal");
      if constexpr (isActive) {
        calculate_result();
      }
      this->result_->action_end = this->node_->now();
      this->goal_handle_->canceled(this->result_);
      this->deactivate();
      return true;
    }
    if (loop_count_ >= this->goal_handle_->get_goal()->goal) {
      RCLCPP_INFO(node_->get_logger(), "Finishing Goal");
      if constexpr (isActive) {
        calculate_result();
      }
      this->result_->action_end = this->node_->now();
      this->goal_handle_->succeed(this->result_);
      this->deactivate();
      return true;
    }
    return false;
  }

  void start() override {
    if constexpr (separate_thread) {
      std::thread([this]() { this->notify(); }).detach();
    } else {
      this->notify();
    }
  }

 protected:
  rclcpp::Node* node_;  // Node reference for logging

  // Count variables
  int count_total_ = 0;
  int count_inside_ = 0;
  int count_outside_ = 0;

  int loop_count_ = 0;

  float x = 0.0;
  float y = 0.0;
};

#endif  // MONTE_CARLO_HPP