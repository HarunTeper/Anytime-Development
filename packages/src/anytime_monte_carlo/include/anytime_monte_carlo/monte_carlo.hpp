#ifndef MONTE_CARLO_HPP
#define MONTE_CARLO_HPP

#include <cstdint>
#include <future>
#include <random>
#include "anytime_monte_carlo/anytime_template.hpp"

// Aliases for better readability
using Anytime = anytime_interfaces::action::Anytime;
using AnytimeGoalHandle = rclcpp_action::ServerGoalHandle<Anytime>;

// Template alias for the Monte Carlo Pi base class
template <bool isBlocking, bool isActive>
using MonteCarloPiBase =
    AnytimeModel<double, Anytime, AnytimeGoalHandle, isBlocking, isActive>;

// Monte Carlo Pi class template
template <bool isBlocking, bool isActive>
class MonteCarloPi : public MonteCarloPiBase<isBlocking, isActive> {
 public:
  // Constructor
  MonteCarloPi() : MonteCarloPiBase<isBlocking, isActive>() {}

  // Reset function
  void reset() override {
    // Reset the count variables
    count_total_ = 0;
    count_inside_ = 0;
    count_outside_ = 0;

    loop_count_ = 0;
  }

  // Blocking function to approximate Pi
  void blocking_function(const std::shared_ptr<AnytimeGoalHandle> goal_handle) {
    if (check_cancel()) {
      this->deactivate();
    }
    if (loop_count_ < goal_handle->get_goal()->goal) {
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
      finish();
      this->deactivate();
    }
  }

  // Non-blocking function to approximate Pi
  void non_blocking_function(
      const std::shared_ptr<AnytimeGoalHandle> goal_handle,
      std::shared_ptr<double> result) {
    (void)goal_handle;
    (void)result;
  }

  bool check_cancel() override {
    // Check if the goal is canceled
    if (this->goal_handle_->is_canceling()) {
      // access the result_ from anytimemodel
      this->result_->result = 4 * (double)count_inside_ / count_total_;

      // Set the result and cancel the goal
      this->goal_handle_->canceled(this->result_);
      return true;
    }
    return false;
  }

  void finish() override {
    // Set the result and succeed the goal
    this->result_->result = 4 * (double)count_inside_ / count_total_;
    this->goal_handle_->succeed(this->result_);
  }

 private:
  // Count variables
  int count_total_ = 0;
  int count_inside_ = 0;
  int count_outside_ = 0;

  int loop_count_ = 0;

  float x = 0.0;
  float y = 0.0;
};

#endif  // MONTE_CARLO_HPP