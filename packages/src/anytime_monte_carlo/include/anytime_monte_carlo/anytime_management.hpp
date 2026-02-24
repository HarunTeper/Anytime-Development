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

    // Initialize reproducible RNG from ROS parameter
    if (!node->has_parameter("random_seed")) {
      node->declare_parameter("random_seed", 42);
    }
    int seed = node->get_parameter("random_seed").as_int();
    rng_ = std::mt19937(static_cast<unsigned int>(seed));

    TRACE_MONTE_CARLO_INIT(node, batch_size, isReactiveProactive);
  }

  // ----------------- Domain-Specific Implementations -----------------

  void compute_single_iteration() override
  {
    RCLCPP_DEBUG(this->node_->get_logger(), "Monte Carlo compute single iteration called");

    x = dist_(rng_);
    y = dist_(rng_);

    if (x * x + y * y <= 1.0f) {
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
  int loop_count_ = 0;

  float x = 0.0f;
  float y = 0.0f;

  // Reproducible RNG
  std::mt19937 rng_;
  std::uniform_real_distribution<float> dist_{0.0f, 1.0f};
};

#endif  // ANYTIME_MANAGEMENT_HPP