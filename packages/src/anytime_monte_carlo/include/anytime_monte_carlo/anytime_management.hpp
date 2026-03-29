// Copyright 2025 Anytime System
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ANYTIME_MONTE_CARLO__ANYTIME_MANAGEMENT_HPP_
#define ANYTIME_MONTE_CARLO__ANYTIME_MANAGEMENT_HPP_

#include <cstdint>
#include <future>
#include <memory>
#include <random>

#include "anytime_core/anytime_base.hpp"
#include "anytime_core/anytime_waitable.hpp"
#include "anytime_interfaces/action/monte_carlo.hpp"
#include "anytime_monte_carlo/tracing.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Aliases for better readability
using Anytime = anytime_interfaces::action::MonteCarlo;
using AnytimeGoalHandle = rclcpp_action::ServerGoalHandle<Anytime>;

// Anytime Management class template
template<bool isReactiveProactive>
class AnytimeManagement : public anytime_core::AnytimeBase<Anytime, AnytimeGoalHandle>
{
public:
  // Constructor
  explicit AnytimeManagement(rclcpp::Node * node, int block_size = 1)
  {
    // Initialize common base class functionality
    this->template initialize_anytime_base<isReactiveProactive>(node, block_size);

    // Initialize reproducible RNG from ROS parameter
    if (!node->has_parameter("random_seed")) {
      node->declare_parameter("random_seed", 42);
    }
    int seed = node->get_parameter("random_seed").as_int();
    rng_ = std::mt19937(static_cast<unsigned int>(seed));

    TRACE_MONTE_CARLO_INIT(node, block_size, isReactiveProactive);
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
    result->result = 4 * static_cast<double>(count_inside_) / count_total_;
    result->iterations = loop_count_;

    // Add additional information to result
    result->block_time = this->average_computation_time_;
    result->block_size = this->block_size_;

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

#endif  // ANYTIME_MONTE_CARLO__ANYTIME_MANAGEMENT_HPP_
