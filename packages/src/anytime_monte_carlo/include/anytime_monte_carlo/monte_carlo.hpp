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
class MonteCarloPi : public AnytimeBase<double, Anytime, AnytimeGoalHandle> {
 public:
  // Constructor
  MonteCarloPi(rclcpp::Node* node) : node_(node) {
    anytime_waitable_ = std::make_shared<AnytimeWaitable>(
        [this]() { this->function(this->goal_handle_); });

    // add the waitable to the node
    node_->get_node_waitables_interface()->add_waitable(
        this->get_anytime_waitable(),
        node_->get_node_base_interface()->get_default_callback_group());

    node_->get_node_waitables_interface()->add_waitable(
        this->get_anytime_finish_waitable(),
        node_->get_node_base_interface()->get_default_callback_group());
  }

  // Blocking function to approximate Pi
  void function(const std::shared_ptr<AnytimeGoalHandle> goal_handle) {
    if (check_cancel()) {
      return;
    }
    if (loop_count_ < goal_handle->get_goal()->goal) {
      RCLCPP_INFO(node_->get_logger(), "Loop count: %d", loop_count_);
      // sample x and y between 0 and 1, and if the length of the vector is
      // greater than one, add count to count_outside, otherwise add to
      // count_inside
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
  }

  bool check_cancel() override {
    // Check if the goal is canceled
    if (this->goal_handle_->is_canceling()) {
      // access the result_ from anytimemodel
      this->result_->result = 4 * (double)count_inside_ / count_total_;

      // Set the result and cancel the goal
      this->goal_handle_->canceled(this->result_);
      this->deactivate();
      return true;
    }
    return false;
  }

  void finish() override {
    RCLCPP_INFO(node_->get_logger(), "Started finish");
    // Set the result and succeed the goal
    this->result_->result = 4 * (double)count_inside_ / count_total_;
    this->goal_handle_->succeed(this->result_);
    this->deactivate();
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