#ifndef MONTE_CARLO_HPP
#define MONTE_CARLO_HPP

#include <cstdint>
#include <future>
#include <random>
#include "anytime_template.hpp"

class MonteCarloPi : public Anytime<uint64_t, double> {
 public:
  // Blocking function to approximate Pi
  double blockingFunction(const uint64_t& numPoints) override {
    uint64_t insideCircle = 0;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    for (uint64_t i = 0; i < numPoints; ++i) {
      double x = dis(gen);
      double y = dis(gen);
      if (x * x + y * y <= 1.0) {
        ++insideCircle;
      }
    }

    return (4.0 * insideCircle) / numPoints;
  }

  // Non-blocking function to approximate Pi
  double nonBlockingFunction(const uint64_t& numPoints) override {
    auto future =
        std::async(std::launch::async, &MonteCarloPi::blockingFunction, this,
                   std::ref(numPoints));
    return future.get();
  }
};

#endif  // MONTE_CARLO_HPP