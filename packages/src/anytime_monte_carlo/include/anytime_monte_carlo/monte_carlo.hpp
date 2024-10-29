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
    AnytimeModel<uint64_t, double, AnytimeGoalHandle, isBlocking, isActive>;

// Monte Carlo Pi class template
template <bool isBlocking, bool isActive>
class MonteCarloPi : public MonteCarloPiBase<isBlocking, isActive> {
 public:
  // Constructor
  MonteCarloPi(std::shared_ptr<AnytimeWaitable> waitable)
      : MonteCarloPiBase<isBlocking, isActive>(waitable) {}

  // Blocking function to approximate Pi
  double blocking_function(const uint64_t& num_samples) {
    (void)num_samples;
    return 0;
  }

  // Non-blocking function to approximate Pi
  void non_blocking_function(const uint64_t& num_samples,
                             std::shared_ptr<double> result) {
    (void)num_samples;
    (void)result;
  }
};

#endif  // MONTE_CARLO_HPP