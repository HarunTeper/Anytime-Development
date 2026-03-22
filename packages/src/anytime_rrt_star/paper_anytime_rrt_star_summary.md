# Paper Summary: Anytime Motion Planning using the RRT*

**Authors:** Sertac Karaman, Matthew R. Walter, Alejandro Perez, Emilio Frazzoli, Seth Teller
**Venue:** IEEE ICRA 2011, Shanghai, China
**Citation:** Karaman et al., "Anytime Motion Planning using the RRT*," 2011 IEEE International Conference on Robotics and Automation, pp. 1478-1483.

---

## 1. Core Idea

Use RRT* as an **anytime** algorithm: quickly find an initial feasible path, then **continuously improve** it toward optimality while the robot executes the current best plan. Two key extensions enable this:
1. **Committed trajectories** — lock an initial portion of the path for execution while replanning the rest
2. **Branch-and-bound pruning** — delete tree nodes that provably cannot lead to better solutions

---

## 2. Problem Formulation

- **State space:** X ⊂ R^d, input space U ⊂ R^m
- **Dynamics:** x_dot(t) = f(x(t), u(t))
- **Obstacle region:** X_obs; free space: X_free = X \ X_obs
- **Goal region:** X_goal ⊂ X
- **Objective:** Find control input u: [0,T] → U yielding a feasible path x(t) ∈ X_free from x_init to X_goal
- **Optimal version:** Minimize cost function c(x) mapping each admissible trajectory to a positive real number

---

## 3. Base RRT* Algorithm (Algorithm 1)

```
RRT*(z_init):
  T ← InitializeTree()
  T ← InsertNode(∅, z_init, T)
  for i = 1 to N do:
    z_rand  ← Sample(i)                         // random state from X_free
    z_nearest ← Nearest(T, z_rand)               // nearest node in tree
    (x_new, u_new, T_new) ← Steer(z_nearest, z_rand)  // extend toward sample
    if ObstacleFree(x_new) then:
      Z_near ← Near(T, z_new, |V|)               // nearby vertices
      z_min  ← ChooseParent(Z_near, z_nearest, z_new, x_new)
      T ← InsertNode(z_min, z_new, T)
      T ← ReWire(T, Z_near, z_min, z_new)
  return T
```

### 3.1 Subroutines

| Subroutine | Description |
|---|---|
| **Sample()** | Random state z_rand ∈ X_free |
| **Dist(z1, z2)** | Cost of optimal trajectory between two states (no obstacles). Without differential constraints = Euclidean distance |
| **Nearest(T, z)** | Return nearest node in tree by distance function |
| **Near(T, z, n)** | Return vertices near z. Radius chosen so ball volume = γ · (log(n)/n)^d where γ is a constant |
| **ObstacleFree(x)** | Check path x: [0,T] → X lies in X_free for all t |
| **Steer(z1, z2)** | Solve for control input driving system from z1 to z2, returns (path, control, time) |
| **InsertNode(z_parent, z_new, T)** | Add z_new to V, create edge to parent, set Cost(z_new) = Cost(parent) + c(edge) |

### 3.2 ChooseParent (Algorithm 2)

```
ChooseParent(Z_near, z_nearest, z_new, x_new):
  z_min ← z_nearest
  c_min ← Cost(z_nearest) + c(x_new)
  for z_near in Z_near:
    (x', u', T') ← Steer(z_near, z_new)
    if ObstacleFree(x') and x'(T') == z_new:
      c' = Cost(z_near) + c(x')
      if c' < Cost(z_new) and c' < c_min:
        z_min ← z_near
        c_min ← c'
  return z_min
```

**Key:** Instead of always using the nearest node as parent, check all nearby nodes and pick the one yielding lowest total cost to z_new.

### 3.3 ReWire (Algorithm 3)

```
ReWire(T, Z_near, z_min, z_new):
  for z_near in Z_near \ {z_min}:
    (x', u', T') ← Steer(z_new, z_near)
    if ObstacleFree(x') and x'(T') == z_near and
       Cost(z_new) + c(x') < Cost(z_near):
      T ← ReConnect(z_new, z_near, T)    // make z_new the new parent of z_near
  return T
```

**Key:** After inserting z_new, check if routing through z_new would reduce cost for any neighbor. If so, rewire the tree.

### 3.4 Near Radius Formula

The neighborhood radius for Near() is defined so the ball contains volume:

```
γ · (log(n) / n)^(1/d)
```

Where:
- `n` = current number of nodes in tree (|V|)
- `d` = dimensionality of state space
- `γ` = a constant (γ_RRT*) derived from the free space volume

The radius **shrinks** as the tree grows, maintaining the asymptotic optimality guarantee while keeping computational cost bounded.

---

## 4. Anytime Extension 1: Committed Trajectory

### 4.1 Two-Phase Online Operation

**Phase 1 — Initial Planning:**
- RRT* runs until the robot must start moving (domain-dependent time, e.g. a few seconds)
- Builds initial tree and finds first feasible solution

**Phase 2 — Iterative Planning (repeats until goal reached):**
1. Select the current best path in the RRT* tree
2. **Commit** to executing the initial portion x: [0, t_com] of the path
3. Delete all branches before x(t_com) and set x(t_com) as the **new tree root**
4. Continue running RRT* to improve the remaining (uncommitted) portion of the tree
5. When the robot reaches the end of the committed trajectory, go to step 1

### 4.2 Key Properties
- The committed trajectory is **shielded from modification** — ensures the robot's current motion is stable
- RRT* continues sampling and rewiring only on the uncommitted portion
- Each cycle: commit → execute → re-root → improve → repeat
- The commit time t_com is a design parameter (trades off stability vs. adaptability)

---

## 5. Anytime Extension 2: Branch-and-Bound Pruning

### 5.1 Cost-to-Go Function (Admissible Heuristic)

For any state z ∈ X_free, define:
- `c*_z` = cost of optimal path from z to X_goal
- `CostToGo(z)` = lower bound on c*_z (must satisfy 0 ≤ CostToGo(z) ≤ c*_z)

This is equivalent to the **admissible heuristic** in A*.

**Paper's choice:** Euclidean distance from z to X_goal divided by maximum speed of the vehicle.

For our 2D geometric case (no dynamics): **CostToGo(z) = Euclidean distance from z to goal**.

### 5.2 Pruning Rule

Let z_min be the node in X_goal with the lowest cost path from root. Then:

```
Upper bound: c_u = Cost(z_min)    // cost of current best solution

Prune set: V' = { z ∈ V | Cost(z) + CostToGo(z) >= c_u }
```

**Periodically delete all nodes in V'** — these nodes cannot possibly lead to a path cheaper than the current best.

### 5.3 Effect
- Focuses tree growth on promising regions
- More effective when CostToGo closely approximates the true optimal cost-to-go
- Applied to both RRT and RRT* in the paper's experiments
- The trivial heuristic CostToGo(z) = 0 is valid but less effective (only prunes nodes whose cost-from-root alone exceeds the best solution)

---

## 6. Differences from Standard RRT

| Aspect | RRT | RRT* |
|---|---|---|
| Parent selection | Always nearest node | Best-cost node among neighbors |
| Rewiring | None | Checks if new node improves cost for neighbors |
| Convergence | Probability zero of reaching optimal | Almost-sure convergence to optimal |
| Online improvement | Local refinements only, gets "stuck" | Structural tree modifications, converges to optimal |
| Computational overhead | Baseline | Slightly higher (near-neighbor search + rewiring), but same order |

---

## 7. Experimental Setup and Results (from paper)

### 7.1 Simulation Environment
- **Vehicle model:** Rear wheel-steered nonholonomic ground vehicle (Dubins vehicle dynamics)
- **Environment:** Bounded region with two polygonal obstacles
- **Task:** Navigate from lower-left to goal region (green box)
- **Steering function:** Dubins curves (4 path classes: combinations of left/straight/right arcs)
- **Monte Carlo runs:** 166 RRT* runs, 191 RRT runs
- **Both planners used:** Branch-and-bound + committed trajectories

### 7.2 Key Results

| Metric | RRT* | RRT |
|---|---|---|
| Mean path length | **23.82 m** | 29.72 m |
| Std deviation | **0.91 m** | 7.48 m |
| Convergence | Tight distribution, near-optimal | Wide distribution, often sub-optimal |

- RRT* consistently finds short, direct paths
- RRT gets "stuck" with sub-optimal tree structure, only makes local improvements
- RRT* exploits execution time for global tree restructuring via rewiring

### 7.3 Real Robot Experiments
- Platform: Robotic forklift, rear wheel-steered
- Environment: 20m × 20m, five obstacles, 1.6m goal region
- RRT* successfully refines paths during execution (e.g., initially goes around obstacle, then discovers shorter path through gap)
- RRT makes only local improvements, often converges to sub-optimal paths

---

## 8. What We Need for Our Implementation

### 8.1 Core Algorithm (must implement)
1. **RRT* main loop** — Algorithm 1 (sample, nearest, steer, obstacle-check, choose-parent, insert, rewire)
2. **ChooseParent** — Algorithm 2 (evaluate all nearby nodes as potential parents)
3. **ReWire** — Algorithm 3 (check if new node provides cheaper paths to neighbors, reconnect if so)
4. **Near radius** — r = γ · (log(n)/n)^(1/d), shrinking with tree size
5. **Cost tracking** — each node stores Cost(z) = cost from root via tree edges
6. **Cost propagation** — when rewiring, propagate updated costs to all descendants

### 8.2 Anytime Extensions (must implement for anytime behavior)
1. **Branch-and-bound pruning:**
   - Track best solution cost (upper bound)
   - CostToGo heuristic = Euclidean distance to goal (for 2D geometric case)
   - Periodically prune nodes where Cost(z) + CostToGo(z) >= best solution cost
2. **Committed trajectory** (maps to our action server's execution model):
   - Initial planning phase → corresponds to our initial block before first feedback
   - Iterative phase → corresponds to continuous block processing with improving results
   - Re-rooting → in our framework, each new goal cycle starts fresh, so committed trajectory maps naturally to the action goal/cancel lifecycle

### 8.3 Steering Function
- Paper uses **Dubins curves** for nonholonomic vehicle
- **For our 2D geometric implementation:** Simple straight-line steering (Steer = move from z1 toward z2 by step_size). This is standard for geometric RRT* in 2D.

### 8.4 Distance Function
- Paper: optimal trajectory cost assuming no obstacles
- **For our case:** Euclidean distance (no differential constraints)

### 8.5 Collision Checking
- Paper: ObstacleFree(x) checks entire path lies in X_free
- **For our case:** Sample points along line segment, check each against occupancy grid

### 8.6 Key Parameters
| Parameter | Description | Paper's approach |
|---|---|---|
| N (iterations) | Number of RRT* iterations | Run until time limit / cancelled |
| γ (gamma) | Near-radius constant | Derived from free space volume, see Karaman & Frazzoli 2011 |
| Step size (η) | Max extension distance per Steer call | Domain-dependent |
| Goal bias | Probability of sampling goal instead of random | Not explicitly mentioned in this paper; common practice is 5-10% |
| t_com | Commit time for trajectory execution | Domain-dependent (seconds for forklift) |

### 8.7 What the Paper Does NOT Cover (we handle separately)
- Specific occupancy grid / PGM map loading
- Block processing architecture (our anytime framework contribution)
- Specific nearest-neighbor data structures (brute-force vs k-d tree)
- Goal biasing strategy details
- Informed sampling (that's Gammell et al. 2014, a different paper)

---

## 9. Implementation Checklist (derived from paper)

- [ ] RRT* main loop with N iterations (Algorithm 1)
- [ ] Sample() — uniform random from X_free (with optional goal bias)
- [ ] Nearest() — find closest node by Euclidean distance
- [ ] Steer() — extend toward sample by step_size (straight line for 2D geometric)
- [ ] ObstacleFree() — check line segment against occupancy grid
- [ ] Near() — find all nodes within radius r = γ · (log(n)/n)^(1/d)
- [ ] ChooseParent() — select lowest-cost parent from nearby nodes (Algorithm 2)
- [ ] InsertNode() — add node with cost = parent cost + edge cost
- [ ] ReWire() — reconnect neighbors through new node if cheaper (Algorithm 3)
- [ ] Cost propagation — update descendant costs after rewiring
- [ ] Branch-and-bound pruning — prune nodes where Cost(z) + dist_to_goal >= best cost
- [ ] Track best path to goal — maintain z_min (lowest-cost node in X_goal)
- [ ] Path extraction — trace from z_min back to root via parent pointers
