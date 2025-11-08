#ifndef ANYTIME_MONTE_CARLO_TRACING_HPP
#define ANYTIME_MONTE_CARLO_TRACING_HPP

#include "anytime_tracing/anytime_tracetools.h"

#include <rclcpp/rclcpp.hpp>

// Helper macros for tracing in anytime_monte_carlo

#define TRACE_MONTE_CARLO_INIT(node, batch_size, is_reactive_proactive)                 \
  ANYTIME_TRACEPOINT(                                                                   \
    monte_carlo_init, static_cast<const void *>(node->get_node_base_interface().get()), \
    batch_size, is_reactive_proactive)

#define TRACE_MONTE_CARLO_ITERATION(node, iteration_num, count_inside, count_total, x, y)    \
  ANYTIME_TRACEPOINT(                                                                        \
    monte_carlo_iteration, static_cast<const void *>(node->get_node_base_interface().get()), \
    iteration_num, count_inside, count_total, x, y)

#define TRACE_MONTE_CARLO_RESULT(node, pi_estimate, total_iterations, count_inside, count_total) \
  ANYTIME_TRACEPOINT(                                                                            \
    monte_carlo_result, static_cast<const void *>(node->get_node_base_interface().get()),        \
    pi_estimate, total_iterations, count_inside, count_total)

#define TRACE_MONTE_CARLO_RESET(node) \
  ANYTIME_TRACEPOINT(                 \
    monte_carlo_reset, static_cast<const void *>(node->get_node_base_interface().get()))

#endif  // ANYTIME_MONTE_CARLO_TRACING_HPP
