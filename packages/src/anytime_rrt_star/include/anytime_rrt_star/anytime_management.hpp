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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <random>
#include <vector>

#include "anytime_core/anytime_base.hpp"
#include "anytime_core/anytime_waitable.hpp"
#include "anytime_interfaces/action/rrt_star.hpp"
#include "anytime_rrt_star/occupancy_grid.hpp"
#include "anytime_rrt_star/tracing.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Aliases for better readability
using Anytime = anytime_interfaces::action::RrtStar;
using AnytimeGoalHandle = rclcpp_action::ServerGoalHandle<Anytime>;

// --- RRT* Data Structures ---

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

    // Load RRT* parameters
    step_size_ = node->get_parameter("step_size").as_double();
    goal_threshold_ = node->get_parameter("goal_threshold").as_double();
    goal_bias_ = node->get_parameter("goal_bias").as_double();
    gamma_rrt_star_ = node->get_parameter("gamma_rrt_star").as_double();
    prune_interval_ = node->get_parameter("prune_interval").as_int();
    convergence_log_interval_ = node->get_parameter("convergence_log_interval").as_int();

    // Load start/goal
    start_.x = node->get_parameter("start_x").as_double();
    start_.y = node->get_parameter("start_y").as_double();
    goal_.x = node->get_parameter("goal_x").as_double();
    goal_.y = node->get_parameter("goal_y").as_double();

    // Load map
    std::string map_yaml_path = node->get_parameter("map_yaml_path").as_string();
    if (!map_yaml_path.empty()) {
      map_.loadFromYaml(map_yaml_path);
      RCLCPP_INFO(
        node->get_logger(), "Loaded map: %dx%d, resolution=%.3f, bounds=[%.1f,%.1f]-[%.1f,%.1f]",
        map_.getWidth(), map_.getHeight(), map_.getResolution(),
        map_.getMinX(), map_.getMinY(), map_.getMaxX(), map_.getMaxY());

      // Set sampling distributions to map bounds
      dist_x_ = std::uniform_real_distribution<double>(map_.getMinX(), map_.getMaxX());
      dist_y_ = std::uniform_real_distribution<double>(map_.getMinY(), map_.getMaxY());

      // Auto-compute gamma if set to 0
      if (gamma_rrt_star_ <= 0.0) {
        double free_area = map_.computeFreeArea();
        // γ_RRT* = 2 * (1 + 1/d)^(1/d) * (μ(X_free) / ζ_d)^(1/d), d=2, ζ_2=π
        gamma_rrt_star_ = 2.0 * std::sqrt(1.5) * std::sqrt(free_area / M_PI);
        RCLCPP_INFO(
          node->get_logger(), "Auto-computed gamma_rrt_star = %.3f (free_area = %.1f m²)",
          gamma_rrt_star_, free_area);
      }
    } else {
      RCLCPP_WARN(node->get_logger(), "No map_yaml_path set, running without obstacle checking");
      // Default sampling range for no-map mode
      dist_x_ = std::uniform_real_distribution<double>(0.0, 30.0);
      dist_y_ = std::uniform_real_distribution<double>(0.0, 15.0);
      if (gamma_rrt_star_ <= 0.0) {
        gamma_rrt_star_ = 50.0;  // reasonable default
      }
    }

    // Validate start/goal are in free space
    if (map_.isLoaded()) {
      if (!map_.isFree(start_.x, start_.y)) {
        RCLCPP_WARN(
          node->get_logger(), "Start position (%.2f, %.2f) is not in free space!",
          start_.x, start_.y);
      }
      if (!map_.isFree(goal_.x, goal_.y)) {
        RCLCPP_WARN(
          node->get_logger(), "Goal position (%.2f, %.2f) is not in free space!",
          goal_.x, goal_.y);
      }
    }

    RCLCPP_INFO(
      node->get_logger(),
      "RRT* params: step=%.2f, goal_thresh=%.2f, goal_bias=%.2f, gamma=%.2f, prune_interval=%d",
      step_size_, goal_threshold_, goal_bias_, gamma_rrt_star_, prune_interval_);
    RCLCPP_INFO(
      node->get_logger(), "RRT* start=(%.2f, %.2f), goal=(%.2f, %.2f)",
      start_.x, start_.y, goal_.x, goal_.y);

    TRACE_RRT_STAR_INIT(node, batch_size, isReactiveProactive);
  }

  // --- Domain-Specific Implementations ---

  void compute_single_iteration() override
  {
    RCLCPP_DEBUG(this->node_->get_logger(), "RRT* compute single iteration called");

    // 1. Sample random point (with goal bias)
    Point2D random_point;
    if (dist_unit_(rng_) < goal_bias_) {
      random_point = goal_;
    } else {
      random_point.x = dist_x_(rng_);
      random_point.y = dist_y_(rng_);
    }

    // 2. Find nearest node in tree
    int nearest_idx = find_nearest(random_point);
    if (nearest_idx < 0) {
      loop_count_++;
      return;
    }

    // 3. Steer towards random point
    Point2D new_point = steer(tree_[nearest_idx].position, random_point);

    // 4. Check collision
    if (!is_collision_free(tree_[nearest_idx].position, new_point)) {
      loop_count_++;
      return;
    }

    // 5. Find nearby nodes for rewiring (Near radius from paper)
    std::vector<int> near_indices = find_near(new_point);

    // 6. ChooseParent (Algorithm 2): pick best parent from nearby nodes
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

    // 7. InsertNode
    RrtNode new_node;
    new_node.position = new_point;
    new_node.parent_index = best_parent;
    new_node.cost_from_start = best_cost;
    int new_idx = static_cast<int>(tree_.size());
    tree_.push_back(new_node);
    tree_[best_parent].children.push_back(new_idx);

    // 8. ReWire (Algorithm 3): rewire nearby nodes through the new node
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
    double dist_to_goal = distance(new_point, goal_);
    if (dist_to_goal < goal_threshold_) {
      if (best_cost < best_path_cost_) {
        best_path_cost_ = best_cost;
        best_goal_node_idx_ = new_idx;
        if (first_solution_iteration_ < 0) {
          first_solution_iteration_ = loop_count_;
          RCLCPP_INFO(
            this->node_->get_logger(),
            "First solution found at iteration %d, cost = %.4f",
            loop_count_, best_path_cost_);
        }
      }
    }

    loop_count_++;

    // 10. Branch-and-bound pruning (periodically)
    if (prune_interval_ > 0 && loop_count_ % prune_interval_ == 0 &&
      best_path_cost_ < std::numeric_limits<double>::infinity())
    {
      prune_tree();
    }

    // Subsampled tracepoint: only emit every convergence_log_interval_ iterations
    if (convergence_log_interval_ > 0 && loop_count_ % convergence_log_interval_ == 0) {
      TRACE_RRT_STAR_ITERATION(
        this->node_, loop_count_, static_cast<int>(tree_.size()), best_path_cost_);
    }
  }

  void populate_feedback(std::shared_ptr<Anytime::Feedback> feedback) override
  {
    feedback->feedback = static_cast<float>(best_path_cost_);
    feedback->tree_size_feedback = static_cast<int32_t>(tree_.size());
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
    result->tree_size = static_cast<int32_t>(tree_.size());
    result->first_solution_iteration = first_solution_iteration_;

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
    first_solution_iteration_ = -1;

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
  int first_solution_iteration_ = -1;

  // --- Map ---
  OccupancyGrid map_;

  // --- Configuration ---
  Point2D start_;
  Point2D goal_;
  double step_size_ = 0.5;
  double goal_threshold_ = 0.5;
  double goal_bias_ = 0.05;
  double gamma_rrt_star_ = 1.0;
  int prune_interval_ = 1000;
  int convergence_log_interval_ = 100;

  // --- RNG ---
  std::mt19937 rng_;
  std::uniform_real_distribution<double> dist_x_{0.0, 30.0};
  std::uniform_real_distribution<double> dist_y_{0.0, 15.0};
  std::uniform_real_distribution<double> dist_unit_{0.0, 1.0};

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
    // Near radius from paper: r = γ_RRT* · (log(n) / n)^(1/d), d=2
    double n = static_cast<double>(tree_.size());
    double radius;
    if (n < 2.0) {
      radius = gamma_rrt_star_;
    } else {
      radius = gamma_rrt_star_ * std::sqrt(std::log(n) / n);
    }
    radius = std::min(radius, step_size_);

    std::vector<int> near;
    for (int i = 0; i < static_cast<int>(tree_.size()); ++i) {
      if (distance(tree_[i].position, point) <= radius) {
        near.push_back(i);
      }
    }
    return near;
  }

  bool is_collision_free(const Point2D & from, const Point2D & to) const
  {
    if (!map_.isLoaded()) {
      return true;  // no map = no obstacles
    }
    return map_.isEdgeFree(from, to);
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

  void prune_tree()
  {
    if (tree_.empty()) {return;}

    // Mark nodes to keep: BFS from root, skip nodes that cannot improve
    std::vector<bool> keep(tree_.size(), false);
    std::queue<int> bfs;
    bfs.push(0);
    keep[0] = true;

    while (!bfs.empty()) {
      int idx = bfs.front();
      bfs.pop();
      for (int child : tree_[idx].children) {
        // Branch-and-bound: Cost(z) + CostToGo(z) < best_cost
        double lower_bound =
          tree_[child].cost_from_start + distance(tree_[child].position, goal_);
        if (lower_bound < best_path_cost_) {
          keep[child] = true;
          bfs.push(child);
        }
      }
    }

    // Always keep the best goal node and its path to root
    if (best_goal_node_idx_ >= 0 && best_goal_node_idx_ < static_cast<int>(tree_.size())) {
      int idx = best_goal_node_idx_;
      while (idx >= 0) {
        keep[idx] = true;
        idx = tree_[idx].parent_index;
      }
    }

    // Count pruned nodes
    int pruned = 0;
    for (size_t i = 0; i < tree_.size(); ++i) {
      if (!keep[i]) {pruned++;}
    }
    if (pruned == 0) {return;}

    // Build index remapping (old index -> new index)
    std::vector<int> new_index(tree_.size(), -1);
    int next_idx = 0;
    for (size_t i = 0; i < tree_.size(); ++i) {
      if (keep[i]) {
        new_index[i] = next_idx++;
      }
    }

    // Build new tree
    std::vector<RrtNode> new_tree;
    new_tree.reserve(next_idx);
    for (size_t i = 0; i < tree_.size(); ++i) {
      if (!keep[i]) {continue;}
      RrtNode node = tree_[i];
      // Remap parent
      if (node.parent_index >= 0) {
        node.parent_index = new_index[node.parent_index];
      }
      // Remap children, keeping only those that survived
      std::vector<int> new_children;
      for (int child : node.children) {
        if (new_index[child] >= 0) {
          new_children.push_back(new_index[child]);
        }
      }
      node.children = new_children;
      new_tree.push_back(node);
    }

    // Update best goal node index
    if (best_goal_node_idx_ >= 0) {
      best_goal_node_idx_ = new_index[best_goal_node_idx_];
    }

    RCLCPP_DEBUG(
      this->node_->get_logger(), "Pruned %d/%zu nodes (best cost=%.4f)",
      pruned, tree_.size(), best_path_cost_);

    tree_ = std::move(new_tree);
  }
};

#endif  // ANYTIME_RRT_STAR__ANYTIME_MANAGEMENT_HPP_
