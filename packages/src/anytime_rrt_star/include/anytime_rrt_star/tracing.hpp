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

#ifndef ANYTIME_RRT_STAR__TRACING_HPP_
#define ANYTIME_RRT_STAR__TRACING_HPP_

#include "anytime_tracing/anytime_tracetools.h"

#include <rclcpp/rclcpp.hpp>

// Placeholder tracing macros for anytime_rrt_star
// TODO: Register domain-specific tracepoints in anytime_tracing once implementation is finalized

#define TRACE_RRT_STAR_INIT(node, batch_size, is_reactive_proactive)
#define TRACE_RRT_STAR_ITERATION(node, iteration_num, tree_size, best_cost)
#define TRACE_RRT_STAR_RESULT(node, best_cost, total_iterations, tree_size)
#define TRACE_RRT_STAR_RESET(node)

#endif  // ANYTIME_RRT_STAR__TRACING_HPP_
