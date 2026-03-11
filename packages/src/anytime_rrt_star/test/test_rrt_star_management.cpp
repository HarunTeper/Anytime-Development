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

#include <gtest/gtest.h>

#include <memory>

#include "anytime_rrt_star/anytime_management.hpp"
#include "rclcpp/rclcpp.hpp"

// Testable subclass that exposes protected members
template<bool isReactiveProactive>
class TestableAnytimeManagement : public AnytimeManagement<isReactiveProactive>
{
public:
  using AnytimeManagement<isReactiveProactive>::AnytimeManagement;
  using AnytimeManagement<isReactiveProactive>::tree_;
  using AnytimeManagement<isReactiveProactive>::loop_count_;
  using AnytimeManagement<isReactiveProactive>::best_path_cost_;
  using AnytimeManagement<isReactiveProactive>::best_goal_node_idx_;
  using AnytimeManagement<isReactiveProactive>::start_;
  using AnytimeManagement<isReactiveProactive>::goal_;
};

class RrtStarManagementTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_rrt_star_node");
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node_;
};

TEST_F(RrtStarManagementTest, Initialization)
{
  auto management = std::make_shared<TestableAnytimeManagement<false>>(node_.get(), 1);
  EXPECT_EQ(management->loop_count_, 0);
  EXPECT_EQ(management->best_goal_node_idx_, -1);
  EXPECT_TRUE(std::isinf(management->best_path_cost_));
}

TEST_F(RrtStarManagementTest, ResetDomainState)
{
  auto management = std::make_shared<TestableAnytimeManagement<false>>(node_.get(), 1);
  management->reset_domain_state();

  EXPECT_EQ(management->loop_count_, 0);
  EXPECT_EQ(management->tree_.size(), 1u);  // Start node
  EXPECT_TRUE(std::isinf(management->best_path_cost_));
  EXPECT_DOUBLE_EQ(management->tree_[0].position.x, management->start_.x);
  EXPECT_DOUBLE_EQ(management->tree_[0].position.y, management->start_.y);
}

TEST_F(RrtStarManagementTest, SingleIteration)
{
  auto management = std::make_shared<TestableAnytimeManagement<false>>(node_.get(), 1);
  management->reset_domain_state();

  management->compute_single_iteration();

  EXPECT_EQ(management->loop_count_, 1);
  // Tree should have grown (start node + potentially new node)
  EXPECT_GE(management->tree_.size(), 1u);
}

TEST_F(RrtStarManagementTest, MultipleIterationsGrowTree)
{
  auto management = std::make_shared<TestableAnytimeManagement<false>>(node_.get(), 1);
  management->reset_domain_state();

  for (int i = 0; i < 100; ++i) {
    management->compute_single_iteration();
  }

  EXPECT_EQ(management->loop_count_, 100);
  // After 100 iterations, tree should have grown significantly
  EXPECT_GT(management->tree_.size(), 10u);
}

TEST_F(RrtStarManagementTest, PopulateFeedback)
{
  auto management = std::make_shared<TestableAnytimeManagement<false>>(node_.get(), 1);
  management->reset_domain_state();

  auto feedback = std::make_shared<Anytime::Feedback>();
  management->populate_feedback(feedback);

  // With no path found, feedback should be infinity (cast to float)
  EXPECT_TRUE(std::isinf(feedback->feedback));
}
