#ifndef ANYTIME_MANAGEMENT_HPP
#define ANYTIME_MANAGEMENT_HPP

#include "anytime_core/anytime_base.hpp"
#include "anytime_core/anytime_waitable.hpp"
#include "anytime_interfaces/action/monte_carlo.hpp"
#include "anytime_monte_carlo/tracing.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <cstdint>
#include <future>
#include <random>

// Aliases for better readability
using Anytime = anytime_interfaces::action::MonteCarlo;
using AnytimeGoalHandle = rclcpp_action::ServerGoalHandle<Anytime>;

// Anytime Management class template
template <bool isReactiveProactive>
class AnytimeManagement : public anytime_core::AnytimeBase<Anytime, AnytimeGoalHandle>
{
public:
  // Constructor
  AnytimeManagement(rclcpp::Node * node, int batch_size = 1)
  {
    // Initialize common base class functionality
    this->template initialize_anytime_base<isReactiveProactive>(node, batch_size);

    TRACE_MONTE_CARLO_INIT(node, batch_size, isReactiveProactive);
  }

  // ----------------- Domain-Specific Implementations -----------------

  void compute_single_iteration() override
  {
    RCLCPP_DEBUG(this->node_->get_logger(), "Monte Carlo compute single iteration called");

    x = (float)rand() / RAND_MAX;
    y = (float)rand() / RAND_MAX;

    if (sqrt(pow(x, 2) + pow(y, 2)) <= 1) {
      count_inside_++;
    }
    count_total_++;
    loop_count_++;

    TRACE_MONTE_CARLO_ITERATION(this->node_, loop_count_, count_inside_, count_total_, x, y);
  }

  void populate_feedback(std::shared_ptr<Anytime::Feedback> feedback) override
  {
    feedback->feedback = count_total_;
    RCLCPP_DEBUG(
      this->node_->get_logger(), "Monte Carlo feedback populated, count total: %d", count_total_);
  }

  void populate_result(std::shared_ptr<Anytime::Result> result) override
  {
    RCLCPP_DEBUG(this->node_->get_logger(), "Monte Carlo result populated");

    // Calculate the result
    result->result = 4 * (double)count_inside_ / count_total_;
    result->iterations = loop_count_;

    // Add additional information to result
    result->batch_time = this->average_computation_time_;
    result->batch_size = this->batch_size_;

    TRACE_MONTE_CARLO_RESULT(
      this->node_, result->result, result->iterations, count_inside_, count_total_);
  }

  void reset_domain_state() override
  {
    TRACE_MONTE_CARLO_RESET(this->node_);
    RCLCPP_DEBUG(this->node_->get_logger(), "Monte Carlo reset domain state called");

    // Reset the count variables
    count_total_ = 0;
    count_inside_ = 0;
    count_outside_ = 0;
    loop_count_ = 0;
  }

  bool should_finish() const override
  {
    return loop_count_ >= this->goal_handle_->get_goal()->goal;
  }

protected:
  // Count variables
  int count_total_ = 0;
  int count_inside_ = 0;
  int count_outside_ = 0;

  int loop_count_ = 0;

  float x = 0.0;
  float y = 0.0;
};

#endif  // ANYTIME_MANAGEMENT_HPP