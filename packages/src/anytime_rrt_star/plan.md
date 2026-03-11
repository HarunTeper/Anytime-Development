# Anytime RRT* Experiment Plan

## 1. Overview

This package (`anytime_rrt_star`) implements an RRT* (Rapidly-exploring Random Tree Star) algorithm as an anytime workload for the Anytime framework. It replaces the Monte Carlo pi-estimation experiment in the scientific paper while preserving the same metrics, reactive/proactive modes, and batch-processing architecture.

**Key parallels with Monte Carlo experiment:**
| Aspect | Monte Carlo | RRT* |
|--------|-------------|------|
| Single iteration | Generate random (x,y), test if inside circle | Sample random point, extend tree, rewire |
| Result (improves over time) | Pi estimate converges to 3.14159... | Path cost converges toward optimal |
| Feedback | Current total sample count | Current best path cost |
| Anytime property | More samples → better estimate | More iterations → shorter path |
| Batch processing | N random samples per batch | N tree extensions per batch |

## 2. What Exists Now (Placeholder)

The current package contains a working skeleton that mirrors the Monte Carlo package:
- **Action interface** (`RrtStar.action`): Same structure as MonteCarlo.action (goal=iterations, result=best cost, feedback=current cost)
- **Management class**: Functional RRT* with random sampling, nearest-neighbor, steering, rewiring, and cost propagation
- **Server/Client**: Identical architecture to Monte Carlo (reactive/proactive modes, batch sizes, cancel timeouts)
- **No obstacles yet**: Collision checking is a placeholder (always returns true)
- **No map loading**: Start/goal and map dimensions are hardcoded

## 3. Available Scientific Work and Repositories

### 3.1 Foundational Papers

1. **Karaman & Frazzoli (2011)** — "Sampling-based Algorithms for Optimal Motion Planning"
   - The original RRT* paper. Proves asymptotic optimality. Defines the rewiring radius formula: `r = γ_RRT* · (log(n)/n)^(1/d)`.
   - Standard experimental setup: 2D unit square with rectangular obstacles.

2. **Gammell, Srinivasa & Barfoot (2014)** — "Informed RRT*: Optimal Sampling-based Path Planning Focused via Direct Sampling of an Admissible Ellipsoidal Heuristic"
   - Improves convergence by sampling within a prolate hyperspheroid once an initial solution is found.
   - Experimental setup: "Random worlds" with 100 independent runs per configuration, 60-second time limits.
   - Source: https://arxiv.org/abs/1404.2334

3. **Karaman, Walter, et al. (2011)** — "Anytime Motion Planning using the RRT*"
   - Specifically about using RRT* as an anytime algorithm with committed trajectories and branch-and-bound pruning.
   - Directly relevant to our anytime framework.

4. **Gammell, Barfoot & Srinivasa (2015)** — "Batch Informed Trees (BIT*)"
   - Combines the strengths of graph-based and tree-based planners.
   - Benchmarks against RRT*, Informed RRT*, and FMT* on random R² worlds.

5. **Naderi, Rajamani & Dippold (2015)** — "RT-RRT*: A Real-Time Path Planning Algorithm Based on RRT"
   - Real-time variant with online tree rewiring. Tested in maze environments.

6. **Ma et al. (ASME IMECE 2025)** — "A Standard Framework for Testing and Benchmarking of 2D RRT Path Planner"
   - Proposes 11 terrain types × 3 difficulty levels = 33 standardized maps.
   - Compares RRT, RRT*, Informed RRT*, LazyRRT, RRT*-Smart.
   - Metrics: computation time, path length, adaptability.

7. **Abbasi et al. (2025)** — "Accelerating RRT* convergence with novel nonuniform and uniform sampling approach"
   - RRT*-NUS evaluated on 384×384 2D scenarios against 6 baselines, 100 trials each, 10,000 sample budget.
   - Published in Nature Scientific Reports.

### 3.2 Key GitHub Repositories

1. **zhm-real/PathPlanning** — Comprehensive collection including RRT, RRT*, Informed RRT*, Anytime RRT*, RRT*-Smart, etc. Python with animations.
   - https://github.com/zhm-real/PathPlanning

2. **motion-planning/rrt-algorithms** — N-dimensional RRT/RRT* with R-tree acceleration. Python with visualization.
   - https://github.com/motion-planning/rrt-algorithms

3. **nikhilchandak/Rapidly-Exploring-Random-Trees** — C++ implementations of RRT, RRT*, and Anytime RRT.
   - https://github.com/nikhilchandak/Rapidly-Exploring-Random-Trees

4. **OMPL (Open Motion Planning Library)** — The gold standard for sampling-based planning. Includes RRT*, Informed RRT*, BIT*, PRM*, etc. with built-in benchmarking infrastructure.
   - https://github.com/ompl/ompl
   - Benchmarking guide: https://ompl.kavrakilab.org/benchmark.html

5. **Bench-MR** — Grid-based motion planning benchmark. Supports occupancy grid maps from images, procedural corridor generation.
   - https://robot-motion.github.io/bench-mr/docs/components/environments/grid/

6. **Kucukcollu/rrt_star_in_ros** — OMPL-based 2D RRT* planner as a ROS node (100×100m map).
   - https://github.com/Kucukcollu/rrt_star_in_ros

7. **KavrakiLab/vamp** — SIMD-accelerated motion planning with comprehensive benchmarks.
   - https://github.com/KavrakiLab/vamp

### 3.3 Benchmarking Tools

- **OMPL Planner Arena**: Interactive exploration of OMPL benchmark results
- **MotionBenchMaker**: Tool to generate and benchmark motion planning datasets (Chamzas et al., 2021)
- **Bench-MR**: Open-source grid-based benchmarking framework

## 4. Possible Directions

### Direction A: Simple 2D Grid with Hardcoded Obstacles
- Use a fixed 2D continuous space (e.g., [0,1]×[0,1] or [0,100]×[0,100])
- Define rectangular/circular obstacles programmatically in C++
- Pros: Self-contained, no external dependencies, fast to iterate, fully reproducible
- Cons: Limited realism, hard to swap maps

### Direction B: Image-Based Occupancy Grid Maps
- Load 2D maps from monochrome PNG/PGM images (black=obstacle, white=free)
- Use standard benchmark maps from literature (bug trap, narrow passage, random worlds, maze)
- Pros: Easy to create/share maps, visual, matches literature conventions
- Cons: Requires image loading library, discretization effects

### Direction C: OMPL Integration
- Use OMPL's state spaces, collision checking, and RRT* implementation as a reference or directly
- Pros: Battle-tested, extensive benchmarking infrastructure, academically credible
- Cons: Heavy dependency, may conflict with our custom anytime framework, OMPL manages its own iteration loop

### Direction D: Procedurally Generated Environments
- Generate random obstacle configurations from seed parameters (obstacle count, size range, density)
- Pros: Systematic parameter sweeps, statistically rigorous, matches "random worlds" from papers
- Cons: May produce degenerate configurations, harder to reproduce specific scenarios

## 5. Standard Benchmark Maps (from Literature)

These are the most commonly used 2D test environments for RRT* evaluation:

1. **Empty/Open Space**: Baseline for measuring raw algorithm overhead
2. **Random Rectangles**: N randomly placed axis-aligned rectangles (Karaman & Frazzoli)
3. **Narrow Passage**: Two large open areas connected by a thin corridor
4. **Bug Trap**: A U-shaped or C-shaped obstacle that traps greedy planners
5. **Maze**: Grid-like corridors requiring many turns
6. **Cluttered**: High obstacle density (30-50% coverage)
7. **Forest**: Many small circular obstacles scattered randomly
8. **Room with Gap**: Single wall with a small gap

## 6. Metrics (Matching Monte Carlo Framework)

### Primary Metrics (via existing tracing/evaluation infrastructure)
- **Iterations completed** per goal cycle (matches MC's `loop_count_`)
- **Best path cost** at cancellation (matches MC's pi estimate — the "result")
- **Batch computation time** (matches MC's `batch_time`)
- **Throughput**: iterations/second (matches MC's throughput)
- **Cancellation delay**: time from cancel request to deactivation

### RRT*-Specific Metrics (new, for paper)
- **Path cost convergence curve**: cost vs. iterations (the key RRT* anytime metric)
- **Tree size**: number of nodes at cancellation
- **Initial solution iteration**: first iteration where a feasible path is found
- **Path cost improvement rate**: Δcost/Δiteration over time

### Experimental Variables (matching Monte Carlo's sweep)
- **Batch sizes**: [1, 16, 64, 256, 1024, 4096, 16384] (tuned for RRT* per-iteration cost)
- **Modes**: reactive, proactive
- **Threading**: single, multi
- **Map complexity**: simple, medium, complex (additional dimension)

## 7. Design Decisions Required

### Decision 1: Map Representation
**Options:**
- **(a) Hardcoded geometric obstacles** (rectangles/circles defined in C++ code or YAML config)
- **(b) Image-based occupancy grid** (load from PNG/PGM, threshold to binary)
- **(c) Hybrid** — hardcoded defaults + optional image override

**Recommendation:** Option (c) — start with hardcoded obstacles for simplicity, add image loading later. This lets us get the experiment running quickly while leaving the door open.

### Decision 2: Map Selection for Experiments
**Options:**
- **(a) Single canonical map** (e.g., "random rectangles" from Karaman & Frazzoli's original paper)
- **(b) Suite of standard maps** (narrow passage, bug trap, maze, cluttered, open)
- **(c) Procedurally generated with controlled parameters** (seed, density, obstacle count)

**Recommendation:** For the paper, option (a) or (b) with 2-3 fixed maps. This keeps the experiment focused. The Monte Carlo experiment had one fixed workload (pi estimation); similarly, RRT* should use a small number of well-understood maps.

### Decision 3: RRT* Variant
**Options:**
- **(a) Standard RRT*** — original Karaman & Frazzoli algorithm
- **(b) Informed RRT*** — ellipsoidal heuristic sampling after initial solution (faster convergence)
- **(c) Both** — standard RRT* as baseline, Informed RRT* as comparison

**Recommendation:** Start with (a) standard RRT* since it is the most well-known and has the clearest anytime convergence property. The standard variant is simpler to implement correctly and is sufficient for demonstrating the anytime framework.

### Decision 4: Collision Checking Approach
**Options:**
- **(a) Point-wise sampling along line segment** (discretize edge, check each point)
- **(b) Analytical line-rectangle/line-circle intersection**
- **(c) Grid-based lookup** (for occupancy grid maps)

**Recommendation:** Option (b) for geometric obstacles (exact, fast) or option (c) for image-based maps.

### Decision 5: Iteration Complexity Tuning
The Monte Carlo experiment uses batch sizes [1024..65536] because each iteration is trivially cheap (2 random numbers + comparison). RRT* iterations are more expensive (nearest-neighbor search is O(n), rewiring is O(k)).

**Options:**
- **(a) Use smaller batch sizes** [1, 16, 64, 256, 1024, 4096]
- **(b) Use spatial indexing (k-d tree)** to keep iteration cost low, enabling larger batches
- **(c) Accept that batch sizes will be smaller** and adjust experiment parameters accordingly

**Recommendation:** Start with (a) and measure. If per-iteration cost is too variable, consider adding a k-d tree later.

### Decision 6: Nearest-Neighbor Data Structure
**Options:**
- **(a) Brute-force linear scan** (O(n) per query, simple, current implementation)
- **(b) k-d tree** (O(log n) per query, standard in RRT* literature)
- **(c) R-tree** (good for range queries needed by rewiring)

**Recommendation:** Start with (a) for correctness, upgrade to (b) if performance requires it. For trees up to ~50k nodes in 2D, brute-force is often acceptable.

### Decision 7: How to Handle "No Path Found"
If the algorithm hasn't found a path to the goal when cancelled:
**Options:**
- **(a) Return infinity** as path cost (current behavior)
- **(b) Return cost to nearest node to goal** (partial progress metric)
- **(c) Ensure map/parameters guarantee a path is always findable within budget**

**Recommendation:** Option (c) — design experiment parameters so that an initial solution is found early (within first 10-20% of iteration budget). This makes the convergence curve meaningful.

### Decision 8: Experiment Scope for Paper
**Options:**
- **(a) Minimal**: One map, same sweep as Monte Carlo (batch size × mode × threading)
- **(b) Moderate**: 2-3 maps, same sweep, + convergence curves
- **(c) Comprehensive**: 5+ maps, full parameter sweep, + RRT* variant comparison

**Recommendation:** Option (b) — enough to show interesting results without exploding the experiment matrix.

## 8. Next Steps

1. **Review this plan** and make design decisions above
2. **Implement obstacle representation** based on Decision 1
3. **Create/select benchmark maps** based on Decision 2
4. **Wire up ROS parameters** for map config, start/goal, step size, rewire radius
5. **Add LTTng tracepoints** for RRT*-specific events
6. **Create experiment runner scripts** (mirroring `experiments/monte_carlo/`)
7. **Create evaluation scripts** with convergence curve plotting
8. **Validate** by running single configurations and checking convergence behavior
9. **Full experiment sweep** with all parameter combinations
