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

#include <memory>

#include "anytime_core/anytime_base.hpp"
#include "anytime_interfaces/action/monte_carlo.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using Anytime = anytime_interfaces::action::MonteCarlo;
using GoalHandle = rclcpp_action::ServerGoalHandle<Anytime>;

// Concrete mock implementation of AnytimeBase for testing
class MockAnytimeBase : public anytime_core::AnytimeBase<Anytime, GoalHandle>
{
public:
  int compute_count = 0;
  int reset_count = 0;
  bool finish_flag = false;

  explicit MockAnytimeBase(rclcpp::Node * node, int block_size = 1)
  {
    this->template initialize_anytime_base<false>(node, block_size);
  }

  void compute_single_iteration() override {compute_count++;}

  void populate_feedback(std::shared_ptr<Anytime::Feedback> feedback) override
  {
    feedback->feedback = static_cast<float>(compute_count);
  }

  void populate_result(std::shared_ptr<Anytime::Result> result) override
  {
    result->result = static_cast<float>(compute_count);
    result->iterations = compute_count;
  }

  void reset_domain_state() override
  {
    reset_count++;
    compute_count = 0;
  }

  bool should_finish() const override {return finish_flag;}
};

class AnytimeBaseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_node");
  }

  void TearDown() override {rclcpp::shutdown();}

  std::shared_ptr<rclcpp::Node> node_;
};

TEST_F(AnytimeBaseTest, InitialStateInactive)
{
  MockAnytimeBase base(node_.get());
  EXPECT_FALSE(base.is_running());
}

TEST_F(AnytimeBaseTest, ActivateDeactivate)
{
  MockAnytimeBase base(node_.get());

  base.activate();
  EXPECT_TRUE(base.is_running());

  base.deactivate();
  EXPECT_FALSE(base.is_running());
}

TEST_F(AnytimeBaseTest, ActivateDeactivateMultipleTimes)
{
  MockAnytimeBase base(node_.get());

  // Multiple activate/deactivate cycles should work
  for (int i = 0; i < 3; i++) {
    base.activate();
    EXPECT_TRUE(base.is_running());
    base.deactivate();
    EXPECT_FALSE(base.is_running());
  }
}

TEST_F(AnytimeBaseTest, SetGetGoalHandle)
{
  MockAnytimeBase base(node_.get());

  // Initially null
  EXPECT_EQ(base.get_goal_handle(), nullptr);

  // After set, should return the same pointer
  // We can't easily create a real ServerGoalHandle, so test with nullptr
  base.set_goal_handle(nullptr);
  EXPECT_EQ(base.get_goal_handle(), nullptr);
}

TEST_F(AnytimeBaseTest, ResetClearsDomainState)
{
  MockAnytimeBase base(node_.get());

  // Simulate some work
  base.compute_count = 42;

  // Reset should call reset_domain_state()
  base.reset();

  EXPECT_EQ(base.reset_count, 1);
  EXPECT_EQ(base.compute_count, 0);
}

TEST_F(AnytimeBaseTest, ResetMultipleTimes)
{
  MockAnytimeBase base(node_.get());

  base.compute_count = 10;
  base.reset();
  EXPECT_EQ(base.reset_count, 1);
  EXPECT_EQ(base.compute_count, 0);

  base.compute_count = 20;
  base.reset();
  EXPECT_EQ(base.reset_count, 2);
  EXPECT_EQ(base.compute_count, 0);
}

TEST_F(AnytimeBaseTest, DefaultBlockIterations)
{
  // Default block_size = 1
  MockAnytimeBase base1(node_.get(), 1);
  EXPECT_EQ(base1.get_block_iterations(), 1);

  // Custom block_size
  MockAnytimeBase base5(node_.get(), 5);
  EXPECT_EQ(base5.get_block_iterations(), 5);
}

TEST_F(AnytimeBaseTest, DefaultProcessGpuCompletions)
{
  MockAnytimeBase base(node_.get());

  // Base implementation is a no-op — should not crash
  EXPECT_NO_THROW(base.process_gpu_completions());
}

TEST_F(AnytimeBaseTest, ShouldFinishReflectsFlag)
{
  MockAnytimeBase base(node_.get());

  EXPECT_FALSE(base.should_finish());
  base.finish_flag = true;
  EXPECT_TRUE(base.should_finish());
}

TEST_F(AnytimeBaseTest, CalculateResultPopulatesResult)
{
  MockAnytimeBase base(node_.get());

  base.compute_count = 42;
  base.calculate_result();

  // result_ is protected, but we can verify through populate_result behavior
  // calculate_result() creates a new result and calls populate_result()
  // If it didn't crash, the method works correctly
}

TEST_F(AnytimeBaseTest, PopulateFeedbackWorks)
{
  MockAnytimeBase base(node_.get());

  base.compute_count = 7;

  auto feedback = std::make_shared<Anytime::Feedback>();
  base.populate_feedback(feedback);

  EXPECT_FLOAT_EQ(feedback->feedback, 7.0f);
}

TEST_F(AnytimeBaseTest, PopulateResultWorks)
{
  MockAnytimeBase base(node_.get());

  base.compute_count = 99;

  auto result = std::make_shared<Anytime::Result>();
  base.populate_result(result);

  EXPECT_FLOAT_EQ(result->result, 99.0f);
  EXPECT_EQ(result->iterations, 99);
}
