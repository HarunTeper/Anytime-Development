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

#ifndef ANYTIME_CORE__TRACING_HPP_
#define ANYTIME_CORE__TRACING_HPP_

#include "anytime_tracing/anytime_tracetools.h"

#include <rclcpp/rclcpp.hpp>

namespace anytime_core
{

// Helper macros for tracing in anytime_core
// These provide a convenient interface for adding tracepoints throughout the codebase

// ==================== Core: AnytimeBase Tracing Helpers ====================

#define TRACE_ANYTIME_BASE_INIT(node, block_size, is_reactive_proactive) \
  ANYTIME_TRACEPOINT( \
    anytime_base_init, static_cast<const void *>(node->get_node_base_interface().get()), \
    block_size, is_reactive_proactive)

#define TRACE_ANYTIME_BASE_ACTIVATE(node) \
  ANYTIME_TRACEPOINT( \
    anytime_base_activate, static_cast<const void *>(node->get_node_base_interface().get()))

#define TRACE_ANYTIME_BASE_DEACTIVATE(node) \
  ANYTIME_TRACEPOINT( \
    anytime_base_deactivate, static_cast<const void *>(node->get_node_base_interface().get()))

#define TRACE_ANYTIME_BASE_RESET(node) \
  ANYTIME_TRACEPOINT( \
    anytime_base_reset, static_cast<const void *>(node->get_node_base_interface().get()))

#define TRACE_REACTIVE_ANYTIME_FUNCTION_ENTRY(node) \
  ANYTIME_TRACEPOINT( \
    reactive_anytime_function_entry, \
    static_cast<const void *>(node->get_node_base_interface().get()))

#define TRACE_REACTIVE_ANYTIME_FUNCTION_EXIT(node, should_finish, should_cancel) \
  ANYTIME_TRACEPOINT( \
    reactive_anytime_function_exit, \
    static_cast<const void *>(node->get_node_base_interface().get()), should_finish, \
    should_cancel)

#define TRACE_PROACTIVE_ANYTIME_FUNCTION_ENTRY(node) \
  ANYTIME_TRACEPOINT( \
    proactive_anytime_function_entry, \
    static_cast<const void *>(node->get_node_base_interface().get()))

#define TRACE_PROACTIVE_ANYTIME_FUNCTION_EXIT(node, should_finish, should_cancel) \
  ANYTIME_TRACEPOINT( \
    proactive_anytime_function_exit, \
    static_cast<const void *>(node->get_node_base_interface().get()), should_finish, \
    should_cancel)

#define TRACE_ANYTIME_COMPUTE_ENTRY(node, block_size) \
  ANYTIME_TRACEPOINT( \
    anytime_compute_entry, static_cast<const void *>(node->get_node_base_interface().get()), \
    block_size)

#define TRACE_ANYTIME_COMPUTE_EXIT( \
    node, iterations_completed, computation_time_ns, average_time_ns) \
  ANYTIME_TRACEPOINT( \
    anytime_compute_exit, static_cast<const void *>(node->get_node_base_interface().get()), \
    iterations_completed, computation_time_ns, average_time_ns)

#define TRACE_ANYTIME_COMPUTE_ITERATION(node, iteration_num) \
  ANYTIME_TRACEPOINT( \
    anytime_compute_iteration, static_cast<const void *>(node->get_node_base_interface().get()), \
    iteration_num)

#define TRACE_ANYTIME_SEND_FEEDBACK_ENTRY(node) \
  ANYTIME_TRACEPOINT( \
    anytime_send_feedback_entry, static_cast<const void *>(node->get_node_base_interface().get()))

#define TRACE_ANYTIME_SEND_FEEDBACK_EXIT(node) \
  ANYTIME_TRACEPOINT( \
    anytime_send_feedback_exit, static_cast<const void *>(node->get_node_base_interface().get()))

#define TRACE_ANYTIME_CALCULATE_RESULT_ENTRY(node) \
  ANYTIME_TRACEPOINT( \
    anytime_calculate_result_entry, \
    static_cast<const void *>(node->get_node_base_interface().get()))

#define TRACE_ANYTIME_CALCULATE_RESULT_EXIT(node) \
  ANYTIME_TRACEPOINT( \
    anytime_calculate_result_exit, \
    static_cast<const void *>(node->get_node_base_interface().get()))

// ==================== Core: AnytimeServer Tracing Helpers ====================

#define TRACE_ANYTIME_SERVER_INIT(node, node_name) \
  ANYTIME_TRACEPOINT( \
    anytime_server_init, static_cast<const void *>(node->get_node_base_interface().get()), \
    node_name)

#define TRACE_ANYTIME_SERVER_HANDLE_GOAL(node, accepted) \
  ANYTIME_TRACEPOINT( \
    anytime_server_handle_goal, static_cast<const void *>(node->get_node_base_interface().get()), \
    accepted)

#define TRACE_ANYTIME_SERVER_HANDLE_CANCEL(node) \
  ANYTIME_TRACEPOINT( \
    anytime_server_handle_cancel, \
    static_cast<const void *>(node->get_node_base_interface().get()))

#define TRACE_ANYTIME_SERVER_HANDLE_ACCEPTED(node) \
  ANYTIME_TRACEPOINT( \
    anytime_server_handle_accepted, \
    static_cast<const void *>(node->get_node_base_interface().get()))

// ==================== Core: AnytimeClient Tracing Helpers ====================

#define TRACE_ANYTIME_CLIENT_INIT(node, node_name) \
  ANYTIME_TRACEPOINT( \
    anytime_client_init, static_cast<const void *>(node->get_node_base_interface().get()), \
    node_name)

#define TRACE_ANYTIME_CLIENT_SEND_GOAL(node) \
  ANYTIME_TRACEPOINT( \
    anytime_client_send_goal, static_cast<const void *>(node->get_node_base_interface().get()))

#define TRACE_ANYTIME_CLIENT_GOAL_RESPONSE(node, accepted) \
  ANYTIME_TRACEPOINT( \
    anytime_client_goal_response, \
    static_cast<const void *>(node->get_node_base_interface().get()), accepted)

#define TRACE_ANYTIME_CLIENT_FEEDBACK(node) \
  ANYTIME_TRACEPOINT( \
    anytime_client_feedback, static_cast<const void *>(node->get_node_base_interface().get()))

#define TRACE_ANYTIME_CLIENT_RESULT(node, result_code) \
  ANYTIME_TRACEPOINT( \
    anytime_client_result, static_cast<const void *>(node->get_node_base_interface().get()), \
    result_code)

#define TRACE_ANYTIME_CLIENT_CANCEL_REQUEST(node) \
  ANYTIME_TRACEPOINT( \
    anytime_client_cancel_request, \
    static_cast<const void *>(node->get_node_base_interface().get()))

#define TRACE_ANYTIME_CLIENT_CANCEL_RESPONSE(node, accepted) \
  ANYTIME_TRACEPOINT( \
    anytime_client_cancel_response, \
    static_cast<const void *>(node->get_node_base_interface().get()), accepted)

// Timing-specific client tracing helpers
#define TRACE_ANYTIME_CLIENT_GOAL_SENT(node, timestamp_ns) \
  ANYTIME_TRACEPOINT( \
    anytime_client_goal_sent, static_cast<const void *>(node->get_node_base_interface().get()), \
    timestamp_ns)

#define TRACE_ANYTIME_CLIENT_CANCEL_SENT(node, timestamp_ns) \
  ANYTIME_TRACEPOINT( \
    anytime_client_cancel_sent, static_cast<const void *>(node->get_node_base_interface().get()), \
    timestamp_ns)

#define TRACE_ANYTIME_CLIENT_GOAL_FINISHED(node, timestamp_ns, result_code) \
  ANYTIME_TRACEPOINT( \
    anytime_client_goal_finished, \
    static_cast<const void *>(node->get_node_base_interface().get()), timestamp_ns, result_code)

}  // namespace anytime_core

#endif  // ANYTIME_CORE__TRACING_HPP_
