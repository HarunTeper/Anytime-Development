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

#include <cmath>
#include <memory>

#include "anytime_rrt_star/occupancy_grid.hpp"
#include "anytime_rrt_star/anytime_management.hpp"
#include "rclcpp/rclcpp.hpp"

// ============================================================
// OccupancyGrid Tests
// ============================================================

class OccupancyGridTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(OccupancyGridTest, DefaultConstruction)
{
  OccupancyGrid grid;
  EXPECT_FALSE(grid.isLoaded());
  EXPECT_EQ(grid.getWidth(), 0);
  EXPECT_EQ(grid.getHeight(), 0);
}

TEST_F(OccupancyGridTest, LoadDepotMap)
{
  OccupancyGrid grid;

  // Try loading from the source tree (test runs from package build dir)
  std::string yaml_path = std::string(TEST_MAP_DIR) + "/depot.yaml";
  ASSERT_NO_THROW(grid.loadFromYaml(yaml_path));
  EXPECT_TRUE(grid.isLoaded());
  EXPECT_EQ(grid.getWidth(), 604);
  EXPECT_EQ(grid.getHeight(), 307);
  EXPECT_DOUBLE_EQ(grid.getResolution(), 0.05);
  EXPECT_DOUBLE_EQ(grid.getMinX(), 0.0);
  EXPECT_DOUBLE_EQ(grid.getMinY(), 0.0);
  EXPECT_NEAR(grid.getMaxX(), 30.2, 0.01);
  EXPECT_NEAR(grid.getMaxY(), 15.35, 0.01);
}

TEST_F(OccupancyGridTest, LoadWarehouseMap)
{
  OccupancyGrid grid;
  std::string yaml_path = std::string(TEST_MAP_DIR) + "/warehouse.yaml";
  ASSERT_NO_THROW(grid.loadFromYaml(yaml_path));
  EXPECT_TRUE(grid.isLoaded());
  EXPECT_EQ(grid.getWidth(), 1006);
  EXPECT_EQ(grid.getHeight(), 1674);
  EXPECT_DOUBLE_EQ(grid.getResolution(), 0.03);
  EXPECT_DOUBLE_EQ(grid.getMinX(), -15.1);
  EXPECT_DOUBLE_EQ(grid.getMinY(), -25.0);
}

TEST_F(OccupancyGridTest, InvalidFileThrows)
{
  OccupancyGrid grid;
  EXPECT_THROW(grid.loadFromYaml("/nonexistent/path.yaml"), std::runtime_error);
}

TEST_F(OccupancyGridTest, DepotCollisionChecking)
{
  OccupancyGrid grid;
  std::string yaml_path = std::string(TEST_MAP_DIR) + "/depot.yaml";
  grid.loadFromYaml(yaml_path);

  // Center of the depot should be free space
  EXPECT_TRUE(grid.isFree(15.0, 7.5));

  // Out-of-bounds should not be free
  EXPECT_FALSE(grid.isFree(-1.0, -1.0));
  EXPECT_FALSE(grid.isFree(100.0, 100.0));
}

TEST_F(OccupancyGridTest, DepotEdgeCollisionChecking)
{
  OccupancyGrid grid;
  std::string yaml_path = std::string(TEST_MAP_DIR) + "/depot.yaml";
  grid.loadFromYaml(yaml_path);

  // Short edge in free space should be free
  Point2D a{15.0, 7.5};
  Point2D b{15.5, 7.5};
  EXPECT_TRUE(grid.isEdgeFree(a, b));

  // Edge going out of bounds should not be free
  Point2D c{0.0, 0.0};
  Point2D d{-1.0, -1.0};
  EXPECT_FALSE(grid.isEdgeFree(c, d));
}

TEST_F(OccupancyGridTest, FreeAreaIsPositive)
{
  OccupancyGrid grid;
  std::string yaml_path = std::string(TEST_MAP_DIR) + "/depot.yaml";
  grid.loadFromYaml(yaml_path);

  double free_area = grid.computeFreeArea();
  EXPECT_GT(free_area, 0.0);
  // Depot is ~30x15m with ~92% free = ~414 m²
  EXPECT_GT(free_area, 100.0);
  EXPECT_LT(free_area, 500.0);
}

// ============================================================
// RRT* Management Tests (without map - no obstacle checking)
// ============================================================

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
  using AnytimeManagement<isReactiveProactive>::first_solution_iteration_;
  using AnytimeManagement<isReactiveProactive>::start_;
  using AnytimeManagement<isReactiveProactive>::goal_;
  using AnytimeManagement<isReactiveProactive>::distance;
  using AnytimeManagement<isReactiveProactive>::find_nearest;
  using AnytimeManagement<isReactiveProactive>::steer;
  using AnytimeManagement<isReactiveProactive>::find_near;
  using AnytimeManagement<isReactiveProactive>::prune_tree;
};

class RrtStarManagementTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_rrt_star_node");
    // Declare parameters the management class expects
    node_->declare_parameter("map_yaml_path", "");
    node_->declare_parameter("start_x", 0.0);
    node_->declare_parameter("start_y", 0.0);
    node_->declare_parameter("goal_x", 10.0);
    node_->declare_parameter("goal_y", 10.0);
    node_->declare_parameter("step_size", 1.0);
    node_->declare_parameter("goal_threshold", 1.0);
    node_->declare_parameter("goal_bias", 0.1);
    node_->declare_parameter("gamma_rrt_star", 50.0);
    node_->declare_parameter("prune_interval", 0);  // disabled for basic tests
    node_->declare_parameter("convergence_log_interval", 100);
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
  EXPECT_EQ(management->first_solution_iteration_, -1);
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
  EXPECT_EQ(management->first_solution_iteration_, -1);
}

TEST_F(RrtStarManagementTest, SingleIteration)
{
  auto management = std::make_shared<TestableAnytimeManagement<false>>(node_.get(), 1);
  management->reset_domain_state();

  management->compute_single_iteration();

  EXPECT_EQ(management->loop_count_, 1);
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
  EXPECT_GT(management->tree_.size(), 10u);
}

TEST_F(RrtStarManagementTest, PopulateFeedback)
{
  auto management = std::make_shared<TestableAnytimeManagement<false>>(node_.get(), 1);
  management->reset_domain_state();

  auto feedback = std::make_shared<Anytime::Feedback>();
  management->populate_feedback(feedback);

  EXPECT_TRUE(std::isinf(feedback->feedback));
}

TEST_F(RrtStarManagementTest, FindsPathWithoutObstacles)
{
  // With no map (no obstacles), goal bias 10%, step_size=1, goal at (10,10),
  // the algorithm should find a path within a reasonable number of iterations
  auto management = std::make_shared<TestableAnytimeManagement<false>>(node_.get(), 1);
  management->reset_domain_state();

  for (int i = 0; i < 5000; ++i) {
    management->compute_single_iteration();
  }

  // Should have found at least one path to goal
  EXPECT_LT(management->best_path_cost_, std::numeric_limits<double>::infinity());
  EXPECT_GE(management->best_goal_node_idx_, 0);
  EXPECT_GE(management->first_solution_iteration_, 0);
  EXPECT_LT(management->first_solution_iteration_, 5000);
}

TEST_F(RrtStarManagementTest, PathCostImprovesOverTime)
{
  auto management = std::make_shared<TestableAnytimeManagement<false>>(node_.get(), 1);
  management->reset_domain_state();

  // Run enough to find initial solution
  for (int i = 0; i < 2000; ++i) {
    management->compute_single_iteration();
  }
  double cost_after_2k = management->best_path_cost_;

  // Run more iterations
  for (int i = 0; i < 8000; ++i) {
    management->compute_single_iteration();
  }
  double cost_after_10k = management->best_path_cost_;

  // Cost should improve (decrease) or stay same with more iterations
  EXPECT_LE(cost_after_10k, cost_after_2k);
}

TEST_F(RrtStarManagementTest, DistanceFunction)
{
  auto management = std::make_shared<TestableAnytimeManagement<false>>(node_.get(), 1);

  Point2D a{0.0, 0.0};
  Point2D b{3.0, 4.0};
  EXPECT_DOUBLE_EQ(management->distance(a, b), 5.0);

  Point2D c{1.0, 1.0};
  EXPECT_NEAR(management->distance(a, c), std::sqrt(2.0), 1e-10);
}

TEST_F(RrtStarManagementTest, SteerFunction)
{
  auto management = std::make_shared<TestableAnytimeManagement<false>>(node_.get(), 1);
  management->reset_domain_state();

  // Steer from origin toward (10,0) with step_size=1
  Point2D from{0.0, 0.0};
  Point2D to{10.0, 0.0};
  Point2D result = management->steer(from, to);

  EXPECT_NEAR(result.x, 1.0, 1e-10);
  EXPECT_NEAR(result.y, 0.0, 1e-10);

  // If target is within step_size, return target exactly
  Point2D close{0.5, 0.0};
  Point2D result2 = management->steer(from, close);
  EXPECT_DOUBLE_EQ(result2.x, 0.5);
  EXPECT_DOUBLE_EQ(result2.y, 0.0);
}

TEST_F(RrtStarManagementTest, FindNearestInTree)
{
  auto management = std::make_shared<TestableAnytimeManagement<false>>(node_.get(), 1);
  management->reset_domain_state();

  // Tree has start node at (0,0). Add a few nodes.
  RrtNode n1;
  n1.position = {5.0, 5.0};
  n1.parent_index = 0;
  management->tree_.push_back(n1);

  RrtNode n2;
  n2.position = {3.0, 3.0};
  n2.parent_index = 0;
  management->tree_.push_back(n2);

  // Nearest to (3.5,3.5) should be n2 at (3,3)
  Point2D query{3.5, 3.5};
  int nearest = management->find_nearest(query);
  EXPECT_EQ(nearest, 2);  // index 2 = n2
}

// ============================================================
// Pruning Tests
// ============================================================

TEST_F(RrtStarManagementTest, PruningRemovesSuboptimalNodes)
{
  auto management = std::make_shared<TestableAnytimeManagement<false>>(node_.get(), 1);
  management->reset_domain_state();

  // Run enough iterations to find a path
  for (int i = 0; i < 5000; ++i) {
    management->compute_single_iteration();
  }

  // Path should be found
  ASSERT_LT(management->best_path_cost_, std::numeric_limits<double>::infinity());

  size_t tree_size_before = management->tree_.size();

  // Manually trigger pruning
  management->prune_tree();

  size_t tree_size_after = management->tree_.size();

  // Pruning should remove some nodes (or at least not crash)
  EXPECT_LE(tree_size_after, tree_size_before);
  // Root should still be present
  EXPECT_GE(management->tree_.size(), 1u);
  // Best path cost should be unchanged
  EXPECT_LT(management->best_path_cost_, std::numeric_limits<double>::infinity());
}

// ============================================================
// RRT* with Map Tests
// ============================================================

class RrtStarWithMapTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_rrt_star_map_node");

    std::string map_yaml = std::string(TEST_MAP_DIR) + "/depot.yaml";
    node_->declare_parameter("map_yaml_path", map_yaml);
    node_->declare_parameter("start_x", 5.0);
    node_->declare_parameter("start_y", 5.0);
    node_->declare_parameter("goal_x", 25.0);
    node_->declare_parameter("goal_y", 10.0);
    node_->declare_parameter("step_size", 0.5);
    node_->declare_parameter("goal_threshold", 0.5);
    node_->declare_parameter("goal_bias", 0.05);
    node_->declare_parameter("gamma_rrt_star", 0.0);  // auto-compute
    node_->declare_parameter("prune_interval", 0);
    node_->declare_parameter("convergence_log_interval", 100);
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node_;
};

TEST_F(RrtStarWithMapTest, InitializesWithMap)
{
  auto management = std::make_shared<TestableAnytimeManagement<false>>(node_.get(), 1);
  management->reset_domain_state();

  EXPECT_EQ(management->tree_.size(), 1u);
  EXPECT_DOUBLE_EQ(management->start_.x, 5.0);
  EXPECT_DOUBLE_EQ(management->start_.y, 5.0);
  EXPECT_DOUBLE_EQ(management->goal_.x, 25.0);
  EXPECT_DOUBLE_EQ(management->goal_.y, 10.0);
}

TEST_F(RrtStarWithMapTest, GrowsTreeWithObstacles)
{
  auto management = std::make_shared<TestableAnytimeManagement<false>>(node_.get(), 1);
  management->reset_domain_state();

  for (int i = 0; i < 500; ++i) {
    management->compute_single_iteration();
  }

  EXPECT_EQ(management->loop_count_, 500);
  // Tree should grow, but some samples will be rejected by collision checking
  EXPECT_GT(management->tree_.size(), 1u);
}

TEST_F(RrtStarWithMapTest, FindsPathOnDepotMap)
{
  auto management = std::make_shared<TestableAnytimeManagement<false>>(node_.get(), 1);
  management->reset_domain_state();

  // Depot map is mostly open, so a path from (5,5) to (25,10) should be found
  for (int i = 0; i < 20000; ++i) {
    management->compute_single_iteration();
  }

  EXPECT_LT(management->best_path_cost_, std::numeric_limits<double>::infinity())
    << "Failed to find path on depot map within 20k iterations";
  EXPECT_GE(management->first_solution_iteration_, 0);

  // Path cost should be roughly the Euclidean distance or a bit more
  double euclidean = std::sqrt(20.0 * 20.0 + 5.0 * 5.0);  // ~20.6m
  EXPECT_GT(management->best_path_cost_, euclidean * 0.9);  // at least near optimal
  EXPECT_LT(management->best_path_cost_, euclidean * 3.0);  // not wildly suboptimal
}

TEST_F(RrtStarWithMapTest, PopulateResultAfterRun)
{
  auto management = std::make_shared<TestableAnytimeManagement<false>>(node_.get(), 1);
  management->reset_domain_state();

  for (int i = 0; i < 10000; ++i) {
    management->compute_single_iteration();
  }

  auto result = std::make_shared<Anytime::Result>();
  management->populate_result(result);

  EXPECT_EQ(result->iterations, 10000);
  EXPECT_EQ(result->batch_size, 1);
  // Result should be the best path cost (may be inf if no path found,
  // or a positive value if path was found)
  EXPECT_GT(result->result, 0.0f);
}
