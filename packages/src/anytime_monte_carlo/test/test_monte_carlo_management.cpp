// Copyright 2025 Anytime System
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "anytime_monte_carlo/anytime_management.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "anytime_interfaces/action/monte_carlo.hpp"

#include <cmath>
#include <cstdlib>
#include <memory>

// Testable subclass that exposes protected members
template <bool isReactiveProactive>
class TestableAnytimeManagement : public AnytimeManagement<isReactiveProactive>
{
public:
  using AnytimeManagement<isReactiveProactive>::AnytimeManagement;

  // Expose protected members for testing
  using AnytimeManagement<isReactiveProactive>::count_total_;
  using AnytimeManagement<isReactiveProactive>::count_inside_;
  using AnytimeManagement<isReactiveProactive>::loop_count_;
};

class MonteCarloManagementTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_mc_node");
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<rclcpp::Node> node_;
};

TEST_F(MonteCarloManagementTest, ConstructorInitializes)
{
  // Reactive mode, batch_size=1
  TestableAnytimeManagement<false> mc(node_.get(), 1);

  EXPECT_EQ(mc.count_total_, 0);
  EXPECT_EQ(mc.count_inside_, 0);
  EXPECT_EQ(mc.loop_count_, 0);
}

TEST_F(MonteCarloManagementTest, ComputeSingleIteration)
{
  TestableAnytimeManagement<false> mc(node_.get(), 1);

  // Seed for reproducibility
  srand(42);

  // Run some iterations
  const int N = 100;
  for (int i = 0; i < N; i++) {
    mc.compute_single_iteration();
  }

  EXPECT_EQ(mc.count_total_, N);
  EXPECT_EQ(mc.loop_count_, N);
}

TEST_F(MonteCarloManagementTest, CountInsideWithinBounds)
{
  TestableAnytimeManagement<false> mc(node_.get(), 1);

  srand(42);

  const int N = 1000;
  for (int i = 0; i < N; i++) {
    mc.compute_single_iteration();
  }

  // count_inside must be <= count_total
  EXPECT_LE(mc.count_inside_, mc.count_total_);
  EXPECT_GE(mc.count_inside_, 0);
}

TEST_F(MonteCarloManagementTest, ResetClearsState)
{
  TestableAnytimeManagement<false> mc(node_.get(), 1);

  srand(42);

  // Run some iterations
  for (int i = 0; i < 50; i++) {
    mc.compute_single_iteration();
  }

  EXPECT_EQ(mc.count_total_, 50);
  EXPECT_EQ(mc.loop_count_, 50);

  // Reset should clear everything
  mc.reset_domain_state();

  EXPECT_EQ(mc.count_total_, 0);
  EXPECT_EQ(mc.count_inside_, 0);
  EXPECT_EQ(mc.loop_count_, 0);
}

TEST_F(MonteCarloManagementTest, PopulateResultCalculatesPi)
{
  TestableAnytimeManagement<false> mc(node_.get(), 1);

  srand(42);

  // Run enough iterations for a reasonable pi estimate
  const int N = 100000;
  for (int i = 0; i < N; i++) {
    mc.compute_single_iteration();
  }

  auto result = std::make_shared<anytime_interfaces::action::MonteCarlo::Result>();
  mc.populate_result(result);

  // Pi estimate should be in a reasonable range
  // 4 * (count_inside / count_total) should approximate pi
  EXPECT_GT(result->result, 2.5f);
  EXPECT_LT(result->result, 3.9f);

  // With 100K iterations, should be closer to pi
  EXPECT_NEAR(result->result, 3.14159, 0.1);

  EXPECT_EQ(result->iterations, N);
}

TEST_F(MonteCarloManagementTest, PopulateFeedback)
{
  TestableAnytimeManagement<false> mc(node_.get(), 1);

  srand(42);

  for (int i = 0; i < 25; i++) {
    mc.compute_single_iteration();
  }

  auto feedback = std::make_shared<anytime_interfaces::action::MonteCarlo::Feedback>();
  mc.populate_feedback(feedback);

  EXPECT_EQ(static_cast<int>(feedback->feedback), 25);
}

TEST_F(MonteCarloManagementTest, SeededReproducibility)
{
  // Run with same seed twice, results should match
  int count_inside_run1 = 0;
  int count_inside_run2 = 0;

  const int N = 1000;

  {
    TestableAnytimeManagement<false> mc(node_.get(), 1);
    srand(12345);
    for (int i = 0; i < N; i++) {
      mc.compute_single_iteration();
    }
    count_inside_run1 = mc.count_inside_;
  }

  {
    TestableAnytimeManagement<false> mc(node_.get(), 1);
    srand(12345);
    for (int i = 0; i < N; i++) {
      mc.compute_single_iteration();
    }
    count_inside_run2 = mc.count_inside_;
  }

  EXPECT_EQ(count_inside_run1, count_inside_run2);
  EXPECT_GT(count_inside_run1, 0);
}

TEST_F(MonteCarloManagementTest, ProactiveModeConstructs)
{
  // Proactive mode should also construct without issues
  TestableAnytimeManagement<true> mc(node_.get(), 4);

  EXPECT_EQ(mc.count_total_, 0);
  EXPECT_EQ(mc.loop_count_, 0);
}

TEST_F(MonteCarloManagementTest, DifferentBatchSizes)
{
  // Various batch sizes should construct correctly
  for (int bs : {1, 5, 10, 100}) {
    TestableAnytimeManagement<false> mc(node_.get(), bs);
    EXPECT_EQ(mc.get_batch_iterations(), bs);
  }
}
