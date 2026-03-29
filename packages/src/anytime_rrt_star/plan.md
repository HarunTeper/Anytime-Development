# Anytime RRT* Experiment Plan

## 1. Overview

This package (`anytime_rrt_star`) implements an RRT* (Rapidly-exploring Random Tree Star) algorithm as an anytime workload for the Anytime framework, based on **Karaman et al. (ICRA 2011) — "Anytime Motion Planning using the RRT*"**. It replaces the Monte Carlo pi-estimation experiment in the scientific paper while preserving the same metrics, reactive/proactive modes, and block-processing architecture.

**Key parallels with Monte Carlo experiment:**
| Aspect | Monte Carlo | RRT* |
|--------|-------------|------|
| Single iteration | Generate random (x,y), test if inside circle | Sample random point, extend tree, rewire |
| Result (improves over time) | Pi estimate converges to 3.14159... | Path cost converges toward optimal |
| Feedback | Current total sample count | Current best path cost |
| Anytime property | More samples → better estimate | More iterations → shorter path |
| Block processing | N random samples per block | N tree extensions per block |

## 2. Reference Paper Summary

**Paper:** Karaman, Walter, Perez, Frazzoli, Teller — "Anytime Motion Planning using the RRT*" (ICRA 2011)

The paper extends standard RRT* with two anytime mechanisms:
1. **Committed trajectories** — lock an initial portion of the path for robot execution while replanning the rest
2. **Branch-and-bound pruning** — delete tree nodes that provably cannot lead to better solutions

### 2.1 Base RRT* Algorithm (Algorithm 1 from paper)

```
RRT*(z_init):
  T ← InitializeTree(); InsertNode(∅, z_init, T)
  for i = 1 to N:
    z_rand   ← Sample(i)                    // random state from X_free
    z_nearest ← Nearest(T, z_rand)           // nearest node in tree
    (x_new, u_new, T_new) ← Steer(z_nearest, z_rand)
    if ObstacleFree(x_new):
      Z_near ← Near(T, z_new, |V|)          // nearby vertices within radius
      z_min  ← ChooseParent(Z_near, z_nearest, z_new, x_new)
      InsertNode(z_min, z_new, T)
      ReWire(T, Z_near, z_min, z_new)
  return T
```

### 2.2 ChooseParent (Algorithm 2)

Among all nearby nodes `Z_near` and the nearest node, pick the parent that yields the lowest `Cost(parent) + c(edge)` to reach `z_new`. This replaces RRT's greedy nearest-node attachment.

### 2.3 ReWire (Algorithm 3)

After inserting `z_new`, check each neighbor in `Z_near`: if routing through `z_new` would reduce that neighbor's cost, rewire the tree (change parent, propagate cost updates to all descendants).

### 2.4 Near Radius Formula

```
r = γ_RRT* · (log(n) / n)^(1/d)
```

Where `n` = number of nodes, `d` = dimensionality (2 for us), `γ_RRT*` is derived from the free space volume. The radius shrinks as the tree grows, maintaining asymptotic optimality.

### 2.5 Branch-and-Bound Pruning

Given current best solution cost `c_best`:
- **CostToGo(z)** = Euclidean distance from z to goal (admissible heuristic for 2D geometric case)
- **Prune** any node z where `Cost(z) + CostToGo(z) >= c_best`

These nodes cannot possibly lead to a cheaper path than the current best. Applied periodically.

### 2.6 Committed Trajectory (not implemented — see mapping below)

The paper's committed trajectory mechanism (lock initial path segment, re-root tree, continue planning) is designed for a robot that's physically moving while planning. Our framework doesn't have a robot executing trajectories, so we map this differently (see Section 3).

## 3. Mapping Paper → Anytime Architecture

### 3.1 Direct Correspondences

| Paper Concept | Anytime Architecture | Notes |
|---|---|---|
| RRT* main loop (Algo 1) | `compute_single_iteration()` | One call = one sample+extend+rewire cycle |
| Block of iterations | Base class block loop | N calls to `compute_single_iteration()` between cancel checks |
| Current best path cost | `populate_feedback()` → `feedback.feedback` | Sent to client after each block |
| Final best path cost | `populate_result()` → `result.result` | Sent when goal completes or is cancelled |
| Iteration count (N) | `goal_handle_->get_goal()->goal` | Client requests N total iterations |
| Tree reset between runs | `reset_domain_state()` | Clear tree, re-add start node |
| Termination | `should_finish()` | `loop_count_ >= goal` |

### 3.2 Paper's Committed Trajectory → Our Goal/Cancel Lifecycle

The paper's two-phase operation maps naturally to our action server:

| Paper Phase | Our Framework |
|---|---|
| **Phase 1 (initial planning):** RRT* runs until robot must move | Client sends goal, server runs block loop |
| **Phase 2 (iterative replanning):** commit, execute, re-root, improve | Client cancels previous goal (gets current best), sends new goal (fresh tree) |
| **Committed trajectory** (shielded from modification) | The result returned on cancel — that was the best path at cancel time |
| **Re-root** (delete branches behind committed segment) | `reset_domain_state()` — each new goal starts fresh |

**Key insight:** Our framework naturally provides anytime behavior without needing the committed trajectory mechanism. The client controls when to "harvest" the current best solution via cancel, and each new goal cycle allows the algorithm to converge further or start fresh.

### 3.3 Branch-and-Bound Pruning → Inside Block Loop

Pruning is implemented inside `compute_single_iteration()`, triggered periodically (e.g., every 1000 iterations). This:
- Removes nodes that cannot improve on the current best solution
- Focuses subsequent sampling on promising regions
- Keeps tree size bounded, improving nearest-neighbor performance
- Only activates after a first solution is found (`best_path_cost_ < infinity`)

## 4. Selected Approach: Image-Based Occupancy Grid Maps from Nav2

- Load 2D maps from PGM images with YAML metadata (standard Nav2/ROS2 format)
- Use real-world Nav2 maps: **depot** and **warehouse**
- Pros: Realistic environments, standard ROS2 format, easy to visualize, representative of actual robot deployments
- Cons: Requires PGM loading (straightforward for grayscale)

## 5. Selected Maps

Two Nav2 maps are used, providing a contrast in size and complexity:

### 5.1 Depot (`maps/depot.pgm`)
- **Dimensions:** 604 × 307 pixels
- **Resolution:** 0.05 m/pixel
- **Real-world size:** 30.2 × 15.4 meters
- **Origin:** (0.0, 0.0)
- **Occupancy:** 3.2% occupied, 92.0% free, 4.8% unknown
- **Character:** Small, mostly open layout with sparse obstacles. Good for short-to-medium path planning. Faster initial solutions, room for path optimization.

### 5.2 Warehouse (`maps/warehouse.pgm`)
- **Dimensions:** 1006 × 1674 pixels
- **Resolution:** 0.03 m/pixel
- **Real-world size:** 30.2 × 50.2 meters
- **Origin:** (-15.1, -25.0)
- **Occupancy:** 1.8% occupied, 84.5% free, 13.7% unknown
- **Character:** Large structured environment with aisles and rows. Long paths requiring navigation through corridors. Demonstrates RRT* anytime convergence well due to the need to refine paths through structured obstacles.

## 6. Design Decisions (All Finalized)

### Decision 1: Map Representation — **DECIDED: Image-based occupancy grid (PGM)**
Load Nav2-format PGM maps with YAML metadata. Collision checking uses grid-based lookup: threshold pixel values using `occupied_thresh` / `free_thresh` from YAML. This is the standard ROS2 approach and avoids any custom obstacle definitions.

### Decision 2: Map Selection — **DECIDED: Depot + Warehouse**
Two Nav2 maps providing a contrast in complexity:
- **Depot** (604×307, 0.05 m/px) — small, open, fast initial solutions
- **Warehouse** (1006×1674, 0.03 m/px) — large, structured aisles, longer paths

### Decision 3: RRT* Variant — **DECIDED: Standard RRT* with branch-and-bound**
Standard RRT* from Karaman & Frazzoli (2011) with the branch-and-bound pruning extension from the anytime paper. No Informed RRT* — the standard variant is sufficient for demonstrating the anytime framework and is the most well-known.

### Decision 4: Collision Checking — **DECIDED: Grid-based lookup**
Convert world coordinates to pixel coordinates, check pixel values against `occupied_thresh` / `free_thresh`. Edge collision checking samples points along the line segment at map resolution intervals.

### Decision 5: Block Sizes — **DECIDED: [1, 16, 64, 256, 1024, 4096, 16384]**
Smaller than Monte Carlo's [1024..65536] because RRT* iterations are more expensive (O(n) nearest-neighbor, O(k) rewiring). Start with these and adjust if needed.

### Decision 6: Nearest-Neighbor — **DECIDED: Brute-force linear scan**
O(n) per query. Simple and correct. For trees up to ~50k nodes in 2D, acceptable. Upgrade to k-d tree only if profiling shows it's a bottleneck.

### Decision 7: No Path Found — **DECIDED: Return infinity + ensure maps are solvable**
Return infinity as path cost if no path found. Choose start/goal positions and step sizes that guarantee a path is found within first 10-20% of iteration budget. Goal bias (5-10%) accelerates initial solution finding.

### Decision 8: Experiment Scope — **DECIDED: Moderate (2 maps, full sweep, convergence curves)**
2 maps × block sizes × modes × threading. Plus convergence curve analysis.

## 7. Implementation Specification

### 7.1 New File: `occupancy_grid.hpp` — PGM Map Loader and Collision Checker

**Purpose:** Load a Nav2 PGM+YAML map, expose world-coordinate collision checking.

**Class: `OccupancyGrid`**

```cpp
class OccupancyGrid {
public:
  // Load from Nav2 YAML + PGM file pair
  bool loadFromYaml(const std::string& yaml_path);

  // World coordinate collision checking
  bool isFree(double world_x, double world_y) const;
  bool isEdgeFree(const Point2D& from, const Point2D& to) const;

  // Map bounds in world coordinates
  double getMinX() const;  // origin_x
  double getMaxX() const;  // origin_x + width * resolution
  double getMinY() const;  // origin_y
  double getMaxY() const;  // origin_y + height * resolution
  double getResolution() const;

private:
  // PGM data
  std::vector<uint8_t> data_;   // raw pixel values (0-255)
  int width_ = 0, height_ = 0;

  // YAML metadata
  double resolution_ = 0.05;
  double origin_x_ = 0.0, origin_y_ = 0.0;
  double occupied_thresh_ = 0.65;
  double free_thresh_ = 0.25;
  bool negate_ = false;

  // Coordinate conversion
  bool worldToGrid(double wx, double wy, int& gx, int& gy) const;
  uint8_t getPixel(int gx, int gy) const;
};
```

**PGM Loading Logic:**
1. Parse YAML to get `image`, `resolution`, `origin`, `occupied_thresh`, `free_thresh`, `negate`, `mode`
2. Open PGM file (P5 binary format): read magic, width, height, maxval, then raw pixel data
3. Store as flat `std::vector<uint8_t>` with row-major order

**Coordinate Conversion (worldToGrid):**
```
grid_x = (world_x - origin_x) / resolution
grid_y = (world_y - origin_y) / resolution
```

Note: PGM files store data top-to-bottom, so `pixel_row = (height - 1) - grid_y`.

**Occupancy Thresholding (Nav2 trinary mode):**
- Normalize pixel value: `p = pixel_value / 255.0` (if `negate`, use `1.0 - p`)
- `p >= occupied_thresh` → occupied (collision)
- `p <= free_thresh` → free
- Otherwise → unknown (treat as occupied for collision checking)

**Edge Collision Checking (`isEdgeFree`):**
- Step along the line from `from` to `to` in increments of `resolution / 2` (half a pixel)
- Check each sampled point with `isFree()`
- Return false as soon as any point is not free

**No external dependencies needed** — PGM is trivial to parse, YAML can be parsed with simple line-by-line string parsing (the Nav2 YAML format is flat key-value pairs, no nested structures).

### 7.2 Updates to `anytime_management.hpp` — Algorithm Corrections

**Changes to constructor:**
1. Declare and load ROS parameters: `map_yaml_path`, `start_x`, `start_y`, `goal_x`, `goal_y`, `step_size`, `goal_threshold`, `goal_bias`, `gamma_rrt_star`, `prune_interval`
2. Create `OccupancyGrid` instance and load map from `map_yaml_path`
3. Set `dist_x_` and `dist_y_` ranges from map bounds (not hardcoded 0-1)
4. Set start/goal from parameters

**New member variables:**
```cpp
OccupancyGrid map_;                    // occupancy grid
double goal_bias_ = 0.05;             // probability of sampling goal (5%)
double gamma_rrt_star_ = 1.0;         // Near radius constant
int prune_interval_ = 1000;           // prune every N iterations
int first_solution_iteration_ = -1;   // track when first path found
```

**Fix `find_near()` — proper Near radius from paper:**
```cpp
// r = γ · (log(n) / n)^(1/d), d=2
double n = static_cast<double>(tree_.size());
double radius = gamma_rrt_star_ * std::sqrt(std::log(n + 1.0) / (n + 1.0));
radius = std::min(radius, step_size_);  // cap at step_size
```

The gamma constant `γ_RRT*` should satisfy `γ > (2(1+1/d))^(1/d) · (μ(X_free)/ζ_d)^(1/d)` where `μ(X_free)` is the Lebesgue measure of free space and `ζ_d` is the volume of the unit d-ball. For d=2: `ζ_2 = π`, so `γ > (2·1.5)^0.5 · (free_area / π)^0.5`. We compute this from the map's free space area.

**Fix `compute_single_iteration()` — add goal bias:**
```cpp
Point2D random_point;
if (dist_goal_bias_(rng_) < goal_bias_) {
  random_point = goal_;  // sample goal directly
} else {
  random_point.x = dist_x_(rng_);
  random_point.y = dist_y_(rng_);
}
```

**Fix `is_collision_free()` — use occupancy grid:**
```cpp
bool is_collision_free(const Point2D& from, const Point2D& to) const {
  return map_.isEdgeFree(from, to);
}
```

**Add branch-and-bound pruning:**
```cpp
void prune_tree() {
  if (best_path_cost_ >= std::numeric_limits<double>::infinity()) return;

  // Mark nodes to keep (BFS from root, skip prunable nodes)
  std::vector<bool> keep(tree_.size(), false);
  std::queue<int> queue;
  queue.push(0);  // root
  keep[0] = true;
  while (!queue.empty()) {
    int idx = queue.front(); queue.pop();
    for (int child : tree_[idx].children) {
      double lower_bound = tree_[child].cost_from_start + distance(tree_[child].position, goal_);
      if (lower_bound < best_path_cost_) {
        keep[child] = true;
        queue.push(child);
      }
    }
  }

  // Rebuild tree with only kept nodes (compact, re-index)
  // ... index remapping logic ...
}
```

Call `prune_tree()` every `prune_interval_` iterations inside `compute_single_iteration()`.

**Fix goal checking — use `goal_threshold_` on the new node itself:**
```cpp
if (distance(new_point, goal_) < goal_threshold_) {
  double cost_to_goal = best_cost + distance(new_point, goal_);
  if (cost_to_goal < best_path_cost_) {
    best_path_cost_ = cost_to_goal;
    best_goal_node_idx_ = new_idx;
    if (first_solution_iteration_ < 0) {
      first_solution_iteration_ = loop_count_;
    }
  }
}
```

**Fix `reset_domain_state()` — also reset `first_solution_iteration_`.**

### 7.3 Updates to `config/server_params.yaml`

```yaml
anytime_server:
  ros__parameters:
    is_reactive_proactive: "reactive"
    multi_threading: false
    block_size: 1024
    random_seed: 42
    log_level: "debug"

    # Map configuration
    map_yaml_path: ""  # path to Nav2 YAML file (e.g., "maps/depot.yaml")

    # Start/goal positions (world coordinates)
    start_x: 5.0
    start_y: 5.0
    goal_x: 25.0
    goal_y: 10.0

    # RRT* algorithm parameters
    step_size: 0.5         # max extension distance per Steer (meters)
    goal_threshold: 0.5    # distance to consider goal reached (meters)
    goal_bias: 0.05        # probability of sampling goal (0.0-1.0)
    gamma_rrt_star: 1.0    # Near radius constant (auto-computed if 0)
    prune_interval: 1000   # branch-and-bound pruning every N iterations
```

### 7.4 Updates to `launch/action_server.launch.py`

Add overridable launch arguments for: `map_yaml_path`, `start_x`, `start_y`, `goal_x`, `goal_y`, `step_size`, `goal_threshold`, `goal_bias`, `gamma_rrt_star`, `prune_interval`.

### 7.5 Updates to `anytime_server.hpp` / `anytime_server.cpp`

Declare the new ROS parameters (`map_yaml_path`, `start_x`, `start_y`, `goal_x`, `goal_y`, `step_size`, `goal_threshold`, `goal_bias`, `gamma_rrt_star`, `prune_interval`) so they are available when the management class reads them.

### 7.6 Updates to `CMakeLists.txt`

- Install the `maps/` directory so map files are accessible at runtime:
  ```cmake
  install(DIRECTORY maps/ DESTINATION share/${PROJECT_NAME}/maps)
  ```

### 7.7 Unit Tests

Update `test/test_rrt_star_management.cpp`:
- Test `OccupancyGrid` loading and collision checking
- Test RRT* with a simple map (or a small synthetic occupancy grid)
- Test that branch-and-bound pruning removes nodes correctly
- Test that Near radius formula produces correct values

## 8. Metrics (Matching Monte Carlo Framework)

### Primary Metrics (via existing tracing/evaluation infrastructure)
- **Iterations completed** per goal cycle (matches MC's `loop_count_`)
- **Best path cost** at cancellation (matches MC's pi estimate — the "result")
- **Block computation time** (matches MC's `block_time`)
- **Throughput**: iterations/second (matches MC's throughput)
- **Cancellation delay**: time from cancel request to deactivation

### RRT*-Specific Metrics (new, for paper)
- **Path cost convergence curve**: cost vs. iterations (the key RRT* anytime metric)
- **Tree size**: number of nodes at cancellation
- **Initial solution iteration**: first iteration where a feasible path is found
- **Path cost improvement rate**: Δcost/Δiteration over time

### Experimental Variables (matching Monte Carlo's sweep)
- **Block sizes**: [1, 16, 64, 256, 1024, 4096, 16384] (tuned for RRT* per-iteration cost)
- **Modes**: reactive, proactive
- **Threading**: single, multi
- **Map**: depot, warehouse (additional dimension vs Monte Carlo)

## 9. Available Scientific Work and Repositories

### 9.1 Foundational Papers

1. **Karaman & Frazzoli (2011)** — "Sampling-based Algorithms for Optimal Motion Planning"
   - The original RRT* paper. Proves asymptotic optimality. Defines the rewiring radius formula.

2. **Karaman, Walter, et al. (2011)** — "Anytime Motion Planning using the RRT*"
   - **THE reference paper for this implementation.** Committed trajectories + branch-and-bound pruning.

3. **Gammell, Srinivasa & Barfoot (2014)** — "Informed RRT*"
   - Ellipsoidal heuristic sampling. Not implementing, but relevant context.

4. **Ma et al. (ASME IMECE 2025)** — "A Standard Framework for Testing and Benchmarking of 2D RRT Path Planner"

5. **Abbasi et al. (2025)** — "Accelerating RRT* convergence with novel nonuniform and uniform sampling approach"

### 9.2 Key GitHub Repositories

1. **zhm-real/PathPlanning** — Python RRT/RRT*/Informed RRT*/Anytime RRT*
2. **motion-planning/rrt-algorithms** — N-dimensional RRT/RRT* with R-tree
3. **nikhilchandak/Rapidly-Exploring-Random-Trees** — C++ RRT/RRT*/Anytime RRT
4. **OMPL** — Gold standard for sampling-based planning (reference, not dependency)

## 10. Implementation Steps

1. ~~Select benchmark maps~~ — **DONE**: depot.pgm and warehouse.pgm in `maps/`
2. **Implement `occupancy_grid.hpp`** — PGM + YAML loader, world-coordinate collision checking
3. **Update `anytime_management.hpp`** — Wire map loader, fix Near radius, add goal bias, add branch-and-bound pruning, load parameters from ROS
4. **Update `anytime_server.hpp/.cpp`** — Declare new ROS parameters
5. **Update `config/server_params.yaml`** — Add all RRT* parameters with sensible defaults
6. **Update `launch/action_server.launch.py`** — Add launch argument overrides for new parameters
7. **Update `CMakeLists.txt`** — Install maps directory
8. **Update unit tests** — Test occupancy grid, RRT* with map, pruning, Near radius
9. **Validate** — Run single configurations, check convergence behavior, ensure initial solution found quickly
10. **Add LTTng tracepoints** — Wire up `tracing.hpp` with real tracepoints (match Monte Carlo pattern)
