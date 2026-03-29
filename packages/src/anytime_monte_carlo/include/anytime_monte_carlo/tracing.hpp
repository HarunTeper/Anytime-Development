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

#ifndef ANYTIME_MONTE_CARLO__TRACING_HPP_
#define ANYTIME_MONTE_CARLO__TRACING_HPP_

#include "anytime_tracing/anytime_tracetools.h"

#include <rclcpp/rclcpp.hpp>

// Helper macros for tracing in anytime_monte_carlo

#define TRACE_MONTE_CARLO_INIT(node, block_size, is_reactive_proactive) \
  ANYTIME_TRACEPOINT( \
    monte_carlo_init, static_cast<const void *>(node->get_node_base_interface().get()), \
    block_size, is_reactive_proactive)

#define TRACE_MONTE_CARLO_ITERATION(node, iteration_num, count_inside, count_total, x, y) \
  ANYTIME_TRACEPOINT( \
    monte_carlo_iteration, static_cast<const void *>(node->get_node_base_interface().get()), \
    iteration_num, count_inside, count_total, x, y)

#define TRACE_MONTE_CARLO_RESULT(node, pi_estimate, total_iterations, count_inside, count_total) \
  ANYTIME_TRACEPOINT( \
    monte_carlo_result, static_cast<const void *>(node->get_node_base_interface().get()), \
    pi_estimate, total_iterations, count_inside, count_total)

#define TRACE_MONTE_CARLO_RESET(node) \
  ANYTIME_TRACEPOINT( \
    monte_carlo_reset, static_cast<const void *>(node->get_node_base_interface().get()))

#endif  // ANYTIME_MONTE_CARLO__TRACING_HPP_
