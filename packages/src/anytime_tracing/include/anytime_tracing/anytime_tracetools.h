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

/** \mainpage anytime_tracing: custom tracing instrumentation for anytime system
 *
 * `anytime_tracing` provides utilities to instrument anytime system components.
 * It provides tracepoints for performance analysis and debugging.
 */

#ifndef ANYTIME_TRACING__ANYTIME_TRACETOOLS_H_
#define ANYTIME_TRACING__ANYTIME_TRACETOOLS_H_

#include "anytime_tracing/config.h"
#include "anytime_tracing/visibility_control.hpp"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifndef ANYTIME_TRACING_DISABLED
/**
 * This allows us to select between two versions of each macro
 * to avoid the 'gnu-zero-variadic-macro-arguments' warning:
 *    1. Only one macro argument for tracepoints without any arguments.
 *    2. Up to 10 macro arguments for tracepoints with up to 9 arguments.
 */
#define _ANYTIME_GET_MACRO(_1, _2, _3, _4, _5, _6, _7, _8, _9, _10, NAME, ...) NAME

#define _ANYTIME_TRACEPOINT_NOARGS(event_name) (anytime_trace_##event_name)()
#define _ANYTIME_TRACEPOINT_ARGS(event_name, ...) (anytime_trace_##event_name)(__VA_ARGS__)
#define _ANYTIME_DECLARE_TRACEPOINT_NOARGS(event_name) \
  ANYTIME_TRACING_PUBLIC void anytime_trace_##event_name();
#define _ANYTIME_DECLARE_TRACEPOINT_ARGS(event_name, ...) \
  ANYTIME_TRACING_PUBLIC void anytime_trace_##event_name(__VA_ARGS__);

#define _ANYTIME_GET_MACRO_TRACEPOINT(...)                                                     \
  _ANYTIME_GET_MACRO(                                                                          \
    __VA_ARGS__, _ANYTIME_TRACEPOINT_ARGS, _ANYTIME_TRACEPOINT_ARGS, _ANYTIME_TRACEPOINT_ARGS, \
    _ANYTIME_TRACEPOINT_ARGS, _ANYTIME_TRACEPOINT_ARGS, _ANYTIME_TRACEPOINT_ARGS,              \
    _ANYTIME_TRACEPOINT_ARGS, _ANYTIME_TRACEPOINT_ARGS, _ANYTIME_TRACEPOINT_ARGS,              \
    _ANYTIME_TRACEPOINT_NOARGS, should_not_be_called_without_any_arguments)
#define _ANYTIME_GET_MACRO_DECLARE_TRACEPOINT(...)                                   \
  _ANYTIME_GET_MACRO(                                                                \
    __VA_ARGS__, _ANYTIME_DECLARE_TRACEPOINT_ARGS, _ANYTIME_DECLARE_TRACEPOINT_ARGS, \
    _ANYTIME_DECLARE_TRACEPOINT_ARGS, _ANYTIME_DECLARE_TRACEPOINT_ARGS,              \
    _ANYTIME_DECLARE_TRACEPOINT_ARGS, _ANYTIME_DECLARE_TRACEPOINT_ARGS,              \
    _ANYTIME_DECLARE_TRACEPOINT_ARGS, _ANYTIME_DECLARE_TRACEPOINT_ARGS,              \
    _ANYTIME_DECLARE_TRACEPOINT_ARGS, _ANYTIME_DECLARE_TRACEPOINT_NOARGS,            \
    should_not_be_called_without_any_arguments)

/// Call a tracepoint.
/**
 * The first argument is mandatory and should be the tracepoint event name.
 * The other arguments should be the tracepoint arguments.
 * This is the preferred method over calling the actual function directly.
 *
 * This macro currently supports up to 9 tracepoint arguments after the event name.
 */
#define ANYTIME_TRACEPOINT(...) _ANYTIME_GET_MACRO_TRACEPOINT(__VA_ARGS__)(__VA_ARGS__)
#define ANYTIME_DECLARE_TRACEPOINT(...) \
  _ANYTIME_GET_MACRO_DECLARE_TRACEPOINT(__VA_ARGS__)(__VA_ARGS__)
#else
#define ANYTIME_TRACEPOINT(...) ((void)(0))
#define ANYTIME_DECLARE_TRACEPOINT(...)
#endif  // ANYTIME_TRACING_DISABLED

#ifdef __cplusplus
extern "C" {
#endif

/// Get tracing compilation status.
/**
 * \return `true` if tracing is enabled, `false` otherwise
 */
ANYTIME_TRACING_PUBLIC bool anytime_trace_compile_status();

// ==================== Core: AnytimeBase ====================

/// `anytime_base_init`
ANYTIME_DECLARE_TRACEPOINT(
  anytime_base_init, const void * node_handle, const int batch_size,
  const bool is_reactive_proactive)

/// `anytime_base_activate`
ANYTIME_DECLARE_TRACEPOINT(anytime_base_activate, const void * node_handle)

/// `anytime_base_deactivate`
ANYTIME_DECLARE_TRACEPOINT(anytime_base_deactivate, const void * node_handle)

/// `anytime_base_reset`
ANYTIME_DECLARE_TRACEPOINT(anytime_base_reset, const void * node_handle)

/// `reactive_anytime_function_entry`
ANYTIME_DECLARE_TRACEPOINT(reactive_anytime_function_entry, const void * node_handle)

/// `reactive_anytime_function_exit`
ANYTIME_DECLARE_TRACEPOINT(
  reactive_anytime_function_exit, const void * node_handle, const bool should_finish,
  const bool should_cancel)

/// `proactive_anytime_function_entry`
ANYTIME_DECLARE_TRACEPOINT(proactive_anytime_function_entry, const void * node_handle)

/// `proactive_anytime_function_exit`
ANYTIME_DECLARE_TRACEPOINT(
  proactive_anytime_function_exit, const void * node_handle, const bool should_finish,
  const bool should_cancel)

/// `anytime_compute_entry`
ANYTIME_DECLARE_TRACEPOINT(anytime_compute_entry, const void * node_handle, const int batch_size)

/// `anytime_compute_exit`
ANYTIME_DECLARE_TRACEPOINT(
  anytime_compute_exit, const void * node_handle, const int iterations_completed,
  const int64_t computation_time_ns, const int64_t average_time_ns)

/// `anytime_compute_iteration`
ANYTIME_DECLARE_TRACEPOINT(
  anytime_compute_iteration, const void * node_handle, const int iteration_num)

/// `anytime_send_feedback_entry`
ANYTIME_DECLARE_TRACEPOINT(anytime_send_feedback_entry, const void * node_handle)

/// `anytime_send_feedback_exit`
ANYTIME_DECLARE_TRACEPOINT(anytime_send_feedback_exit, const void * node_handle)

/// `anytime_calculate_result_entry`
ANYTIME_DECLARE_TRACEPOINT(anytime_calculate_result_entry, const void * node_handle)

/// `anytime_calculate_result_exit`
ANYTIME_DECLARE_TRACEPOINT(anytime_calculate_result_exit, const void * node_handle)

// ==================== Core: AnytimeServer ====================

/// `anytime_server_init`
ANYTIME_DECLARE_TRACEPOINT(anytime_server_init, const void * node_handle, const char * node_name)

/// `anytime_server_handle_goal`
ANYTIME_DECLARE_TRACEPOINT(
  anytime_server_handle_goal, const void * node_handle, const bool accepted)

/// `anytime_server_handle_cancel`
ANYTIME_DECLARE_TRACEPOINT(anytime_server_handle_cancel, const void * node_handle)

/// `anytime_server_handle_accepted`
ANYTIME_DECLARE_TRACEPOINT(anytime_server_handle_accepted, const void * node_handle)

// ==================== Core: AnytimeClient ====================

/// `anytime_client_init`
ANYTIME_DECLARE_TRACEPOINT(anytime_client_init, const void * node_handle, const char * node_name)

/// `anytime_client_send_goal`
ANYTIME_DECLARE_TRACEPOINT(anytime_client_send_goal, const void * node_handle)

/// `anytime_client_goal_response`
ANYTIME_DECLARE_TRACEPOINT(
  anytime_client_goal_response, const void * node_handle, const bool accepted)

/// `anytime_client_feedback`
ANYTIME_DECLARE_TRACEPOINT(anytime_client_feedback, const void * node_handle)

/// `anytime_client_result`
ANYTIME_DECLARE_TRACEPOINT(anytime_client_result, const void * node_handle, const int result_code)

/// `anytime_client_cancel_request`
ANYTIME_DECLARE_TRACEPOINT(anytime_client_cancel_request, const void * node_handle)

/// `anytime_client_cancel_response`
ANYTIME_DECLARE_TRACEPOINT(
  anytime_client_cancel_response, const void * node_handle, const bool accepted)

/// `anytime_client_goal_sent` - Timing-specific event
ANYTIME_DECLARE_TRACEPOINT(
  anytime_client_goal_sent, const void * node_handle, const int64_t timestamp_ns)

/// `anytime_client_cancel_sent` - Timing-specific event
ANYTIME_DECLARE_TRACEPOINT(
  anytime_client_cancel_sent, const void * node_handle, const int64_t timestamp_ns)

/// `anytime_client_goal_finished` - Timing-specific event
ANYTIME_DECLARE_TRACEPOINT(
  anytime_client_goal_finished, const void * node_handle, const int64_t timestamp_ns,
  const int result_code)

// ==================== Monte Carlo ====================

/// `monte_carlo_init`
ANYTIME_DECLARE_TRACEPOINT(
  monte_carlo_init, const void * node_handle, const int batch_size,
  const bool is_reactive_proactive)

/// `monte_carlo_iteration`
ANYTIME_DECLARE_TRACEPOINT(
  monte_carlo_iteration, const void * node_handle, const int iteration_num, const int count_inside,
  const int count_total, const float x, const float y)

/// `monte_carlo_result`
ANYTIME_DECLARE_TRACEPOINT(
  monte_carlo_result, const void * node_handle, const double pi_estimate,
  const int total_iterations, const int count_inside, const int count_total)

/// `monte_carlo_reset`
ANYTIME_DECLARE_TRACEPOINT(monte_carlo_reset, const void * node_handle)

// ==================== YOLO ====================

/// `yolo_init`
ANYTIME_DECLARE_TRACEPOINT(
  yolo_init, const void * node_handle, const int batch_size, const bool is_reactive_proactive,
  const bool is_sync_async, const char * weights_path)

/// `yolo_layer_start`
ANYTIME_DECLARE_TRACEPOINT(yolo_layer_start, const void * node_handle, const int layer_num)

/// `yolo_layer_end`
ANYTIME_DECLARE_TRACEPOINT(yolo_layer_end, const void * node_handle, const int layer_num)

/// `yolo_exit_calculation_start`
ANYTIME_DECLARE_TRACEPOINT(
  yolo_exit_calculation_start, const void * node_handle, const int layer_num)

/// `yolo_exit_calculation_end`
ANYTIME_DECLARE_TRACEPOINT(
  yolo_exit_calculation_end, const void * node_handle, const int layer_num,
  const int num_detections)

/// `yolo_detection`
ANYTIME_DECLARE_TRACEPOINT(
  yolo_detection, const void * node_handle, const int layer_num, const int detection_id,
  const float confidence, const int class_id, const float bbox_x, const float bbox_y,
  const float bbox_width, const float bbox_height)

/// `yolo_result`
ANYTIME_DECLARE_TRACEPOINT(
  yolo_result, const void * node_handle, const int processed_layers,
  const int result_processed_layers, const int total_detections)

/// `yolo_reset`
ANYTIME_DECLARE_TRACEPOINT(yolo_reset, const void * node_handle)

/// `yolo_image_processed`
ANYTIME_DECLARE_TRACEPOINT(
  yolo_image_processed, const void * node_handle, const int image_width, const int image_height)

/// `yolo_cuda_callback`
ANYTIME_DECLARE_TRACEPOINT(yolo_cuda_callback, const void * node_handle, const int processed_layers)

// ==================== Interference ====================

/// `interference_timer_init`
ANYTIME_DECLARE_TRACEPOINT(
  interference_timer_init, const void * node_handle, const int timer_period_ms,
  const int execution_time_ms)

/// `interference_timer_callback_entry`
ANYTIME_DECLARE_TRACEPOINT(
  interference_timer_callback_entry, const void * node_handle, const uint64_t execution_count)

/// `interference_timer_callback_exit`
ANYTIME_DECLARE_TRACEPOINT(
  interference_timer_callback_exit, const void * node_handle, const uint64_t execution_count,
  const int64_t actual_duration_ns)

#ifdef __cplusplus
}
#endif

#endif  // ANYTIME_TRACING__ANYTIME_TRACETOOLS_H_
