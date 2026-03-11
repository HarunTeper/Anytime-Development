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

#ifndef ANYTIME_RRT_STAR__ANYTIME_MANAGEMENT_HPP_
#define ANYTIME_RRT_STAR__ANYTIME_MANAGEMENT_HPP_

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <random>
#include <vector>

#include "anytime_core/anytime_base.hpp"
#include "anytime_core/anytime_waitable.hpp"
#include "anytime_interfaces/action/rrt_star.hpp"
#include "anytime_rrt_star/tracing.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Aliases for better readability
using Anytime = anytime_interfaces::action::RrtStar;
using AnytimeGoalHandle = rclcpp_action::ServerGoalHandle<Anytime>;

// --- RRT* Data Structures ---

struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

struct RrtNode
{
  Point2D position;
  int parent_index = -1;
  double cost_from_start = 0.0;
  std::vector<int> children;
};

// --- Anytime Management class template for RRT* ---
template<bool isReactiveProactive>
class AnytimeManagement : public anytime_core::AnytimeBase<Anytime, AnytimeGoalHandle>
{
public:
  explicit AnytimeManagement(rclcpp::Node * node, int batch_size = 1)
  {
    this->template initialize_anytime_base<isReactiveProactive>(node, batch_size);

    // Initialize reproducible RNG from ROS parameter
    if (!node->has_parameter("random_seed")) {
      node->declare_parameter("random_seed", 42);
    }
    int seed = node->get_parameter("random_seed").as_int();
    rng_ = std::mt19937(static_cast<unsigned int>(seed));

    // TODO: Load map/obstacle configuration from parameters
    // TODO: Load start/goal positions from parameters
    // For now, use hardcoded defaults (to be replaced by parameter-driven config)
    start_ = {0.0, 0.0};
    goal_ = {1.0, 1.0};
    map_width_ = 1.0;
    map_height_ = 1.0;
    step_size_ = 0.05;
    goal_threshold_ = 0.05;
    rewire_radius_ = 0.1;

    TRACE_RRT_STAR_INIT(node, batch_size, isReactiveProactive);
  }

  // --- Domain-Specific Implementations ---

  void compute_single_iteration() override
  {
    RCLCPP_DEBUG(this->node_->get_logger(), "RRT* compute single iteration called");

    // 1. Sample random point
    Point2D random_point;
    random_point.x = dist_x_(rng_);
    random_point.y = dist_y_(rng_);

    // 2. Find nearest node in tree
    int nearest_idx = find_nearest(random_point);
    if (nearest_idx < 0) {
      loop_count_++;
      return;
    }

    // 3. Steer towards random point
    Point2D new_point = steer(tree_[nearest_idx].position, random_point);

    // 4. Check collision (placeholder - no obstacles yet)
    if (!is_collision_free(tree_[nearest_idx].position, new_point)) {
      loop_count_++;
      return;
    }

    // 5. Find nearby nodes for rewiring
    std::vector<int> near_indices = find_near(new_point);

    // 6. Choose best parent from nearby nodes
    int best_parent = nearest_idx;
    double best_cost = tree_[nearest_idx].cost_from_start + distance(
      tree_[nearest_idx].position, new_point);

    for (int idx : near_indices) {
      double candidate_cost = tree_[idx].cost_from_start + distance(
        tree_[idx].position, new_point);
      if (candidate_cost < best_cost && is_collision_free(tree_[idx].position, new_point)) {
        best_cost = candidate_cost;
        best_parent = idx;
      }
    }

    // 7. Add new node
    RrtNode new_node;
    new_node.position = new_point;
    new_node.parent_index = best_parent;
    new_node.cost_from_start = best_cost;
    int new_idx = static_cast<int>(tree_.size());
    tree_.push_back(new_node);
    tree_[best_parent].children.push_back(new_idx);

    // 8. Rewire nearby nodes through the new node
    for (int idx : near_indices) {
      if (idx == best_parent) {continue;}
      double new_cost = best_cost + distance(new_point, tree_[idx].position);
      if (new_cost < tree_[idx].cost_from_start &&
        is_collision_free(new_point, tree_[idx].position))
      {
        // Remove from old parent's children
        auto & old_children = tree_[tree_[idx].parent_index].children;
        old_children.erase(
          std::remove(old_children.begin(), old_children.end(), idx),
          old_children.end());
        // Rewire
        tree_[idx].parent_index = new_idx;
        tree_[idx].cost_from_start = new_cost;
        tree_[new_idx].children.push_back(idx);
        propagate_cost_update(idx);
      }
    }

    // 9. Check if we reached the goal
    if (distance(new_point, goal_) < goal_threshold_) {
      if (best_cost + distance(new_point, goal_) < best_path_cost_) {
        best_path_cost_ = best_cost + distance(new_point, goal_);
        best_goal_node_idx_ = new_idx;
      }
    }

    loop_count_++;

    TRACE_RRT_STAR_ITERATION(
      this->node_, loop_count_, static_cast<int>(tree_.size()), best_path_cost_);
  }

  void populate_feedback(std::shared_ptr<Anytime::Feedback> feedback) override
  {
    feedback->feedback = static_cast<float>(best_path_cost_);
    RCLCPP_DEBUG(
      this->node_->get_logger(), "RRT* feedback: best cost = %f, tree size = %zu",
      best_path_cost_, tree_.size());
  }

  void populate_result(std::shared_ptr<Anytime::Result> result) override
  {
    RCLCPP_DEBUG(this->node_->get_logger(), "RRT* result populated");

    result->result = static_cast<float>(best_path_cost_);
    result->iterations = loop_count_;
    result->batch_time = this->average_computation_time_;
    result->batch_size = this->batch_size_;

    TRACE_RRT_STAR_RESULT(
      this->node_, best_path_cost_, loop_count_, static_cast<int>(tree_.size()));
  }

  void reset_domain_state() override
  {
    TRACE_RRT_STAR_RESET(this->node_);
    RCLCPP_DEBUG(this->node_->get_logger(), "RRT* reset domain state called");

    tree_.clear();
    loop_count_ = 0;
    best_path_cost_ = std::numeric_limits<double>::infinity();
    best_goal_node_idx_ = -1;

    // Re-add start node
    RrtNode start_node;
    start_node.position = start_;
    start_node.parent_index = -1;
    start_node.cost_from_start = 0.0;
    tree_.push_back(start_node);
  }

  bool should_finish() const override
  {
    return loop_count_ >= this->goal_handle_->get_goal()->goal;
  }

protected:
  // --- Tree state ---
  std::vector<RrtNode> tree_;
  int loop_count_ = 0;
  double best_path_cost_ = std::numeric_limits<double>::infinity();
  int best_goal_node_idx_ = -1;

  // --- Map configuration ---
  Point2D start_;
  Point2D goal_;
  double map_width_ = 1.0;
  double map_height_ = 1.0;
  double step_size_ = 0.05;
  double goal_threshold_ = 0.05;
  double rewire_radius_ = 0.1;

  // TODO: Obstacle representation (to be decided - see plan.md)
  // std::vector<Obstacle> obstacles_;

  // --- RNG ---
  std::mt19937 rng_;
  std::uniform_real_distribution<double> dist_x_{0.0, 1.0};
  std::uniform_real_distribution<double> dist_y_{0.0, 1.0};

  // --- Helper functions ---

  double distance(const Point2D & a, const Point2D & b) const
  {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  int find_nearest(const Point2D & point) const
  {
    if (tree_.empty()) {return -1;}
    int nearest = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < static_cast<int>(tree_.size()); ++i) {
      double d = distance(tree_[i].position, point);
      if (d < min_dist) {
        min_dist = d;
        nearest = i;
      }
    }
    return nearest;
  }

  Point2D steer(const Point2D & from, const Point2D & to) const
  {
    double d = distance(from, to);
    if (d <= step_size_) {
      return to;
    }
    double ratio = step_size_ / d;
    return {from.x + ratio * (to.x - from.x), from.y + ratio * (to.y - from.y)};
  }

  std::vector<int> find_near(const Point2D & point) const
  {
    // RRT* radius: gamma * (log(n) / n)^(1/d) where d=2
    double n = static_cast<double>(tree_.size());
    double radius = std::min(
      rewire_radius_ * std::sqrt(std::log(n + 1.0) / (n + 1.0)),
      step_size_);
    // Use fixed radius as fallback for small trees
    if (tree_.size() < 10) {
      radius = rewire_radius_;
    }

    std::vector<int> near;
    for (int i = 0; i < static_cast<int>(tree_.size()); ++i) {
      if (distance(tree_[i].position, point) < radius) {
        near.push_back(i);
      }
    }
    return near;
  }

  bool is_collision_free(const Point2D & from, const Point2D & to) const
  {
    // TODO: Implement actual collision checking against obstacles
    // Placeholder: no obstacles, always collision-free
    (void)from;
    (void)to;
    return true;
  }

  void propagate_cost_update(int node_idx)
  {
    for (int child_idx : tree_[node_idx].children) {
      tree_[child_idx].cost_from_start =
        tree_[node_idx].cost_from_start +
        distance(tree_[node_idx].position, tree_[child_idx].position);
      propagate_cost_update(child_idx);
    }
  }
};

#endif  // ANYTIME_RRT_STAR__ANYTIME_MANAGEMENT_HPP_
