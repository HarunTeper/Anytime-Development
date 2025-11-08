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

// Provide fake header guard for cpplint
#undef ANYTIME_TRACING__TP_CALL_H_
#ifndef ANYTIME_TRACING__TP_CALL_H_
#define ANYTIME_TRACING__TP_CALL_H_

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER anytime

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "anytime_tracing/tp_call.h"

#if !defined(_ANYTIME_TRACING__TP_CALL_H_) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _ANYTIME_TRACING__TP_CALL_H_

#include <lttng/tracepoint.h>
#include <stdbool.h>
#include <stdint.h>

// ==================== Core: AnytimeBase ====================

// AnytimeBase lifecycle events
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, anytime_base_init,
  TP_ARGS(
    const void *, node_handle_arg, const int, batch_size_arg, const bool,
    is_reactive_proactive_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_integer(int, batch_size, batch_size_arg)
                ctf_integer(bool, is_reactive_proactive, is_reactive_proactive_arg)
                  ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, anytime_base_activate, TP_ARGS(const void *, node_handle_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, anytime_base_deactivate, TP_ARGS(const void *, node_handle_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, anytime_base_reset, TP_ARGS(const void *, node_handle_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_string(version, anytime_tracing_VERSION)))

// Anytime function execution
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, reactive_anytime_function_entry, TP_ARGS(const void *, node_handle_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, reactive_anytime_function_exit,
  TP_ARGS(
    const void *, node_handle_arg, const bool, should_finish_arg, const bool, should_cancel_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_integer(bool, should_finish, should_finish_arg)
                ctf_integer(bool, should_cancel, should_cancel_arg)
                  ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, proactive_anytime_function_entry, TP_ARGS(const void *, node_handle_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, proactive_anytime_function_exit,
  TP_ARGS(
    const void *, node_handle_arg, const bool, should_finish_arg, const bool, should_cancel_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_integer(bool, should_finish, should_finish_arg)
                ctf_integer(bool, should_cancel, should_cancel_arg)
                  ctf_string(version, anytime_tracing_VERSION)))

// Compute operations
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, anytime_compute_entry,
  TP_ARGS(const void *, node_handle_arg, const int, batch_size_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_integer(int, batch_size, batch_size_arg)
                ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, anytime_compute_exit,
  TP_ARGS(
    const void *, node_handle_arg, const int, iterations_completed_arg, const int64_t,
    computation_time_ns_arg, const int64_t, average_time_ns_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_integer(int, iterations_completed, iterations_completed_arg)
                ctf_integer(int64_t, computation_time_ns, computation_time_ns_arg)
                  ctf_integer(int64_t, average_time_ns, average_time_ns_arg)
                    ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, anytime_compute_iteration,
  TP_ARGS(const void *, node_handle_arg, const int, iteration_num_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_integer(int, iteration_num, iteration_num_arg)
                ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, anytime_send_feedback_entry, TP_ARGS(const void *, node_handle_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, anytime_send_feedback_exit, TP_ARGS(const void *, node_handle_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, anytime_calculate_result_entry, TP_ARGS(const void *, node_handle_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, anytime_calculate_result_exit, TP_ARGS(const void *, node_handle_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_string(version, anytime_tracing_VERSION)))

// ==================== Core: AnytimeServer ====================

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, anytime_server_init,
  TP_ARGS(const void *, node_handle_arg, const char *, node_name_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_string(node_name, node_name_arg) ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, anytime_server_handle_goal,
  TP_ARGS(const void *, node_handle_arg, const bool, accepted_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_integer(bool, accepted, accepted_arg)
                ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, anytime_server_handle_cancel, TP_ARGS(const void *, node_handle_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, anytime_server_handle_accepted, TP_ARGS(const void *, node_handle_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_string(version, anytime_tracing_VERSION)))

// ==================== Core: AnytimeClient ====================

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, anytime_client_init,
  TP_ARGS(const void *, node_handle_arg, const char *, node_name_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_string(node_name, node_name_arg) ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, anytime_client_send_goal, TP_ARGS(const void *, node_handle_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, anytime_client_goal_response,
  TP_ARGS(const void *, node_handle_arg, const bool, accepted_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_integer(bool, accepted, accepted_arg)
                ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, anytime_client_feedback, TP_ARGS(const void *, node_handle_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, anytime_client_result,
  TP_ARGS(const void *, node_handle_arg, const int, result_code_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_integer(int, result_code, result_code_arg)
                ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, anytime_client_cancel_request, TP_ARGS(const void *, node_handle_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, anytime_client_cancel_response,
  TP_ARGS(const void *, node_handle_arg, const bool, accepted_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_integer(bool, accepted, accepted_arg)
                ctf_string(version, anytime_tracing_VERSION)))

// ==================== Monte Carlo ====================

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, monte_carlo_init,
  TP_ARGS(
    const void *, node_handle_arg, const int, batch_size_arg, const bool,
    is_reactive_proactive_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_integer(int, batch_size, batch_size_arg)
                ctf_integer(bool, is_reactive_proactive, is_reactive_proactive_arg)
                  ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, monte_carlo_iteration,
  TP_ARGS(
    const void *, node_handle_arg, const int, iteration_num_arg, const int, count_inside_arg,
    const int, count_total_arg, const float, x_arg, const float, y_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_integer(int, iteration_num, iteration_num_arg)
                ctf_integer(int, count_inside, count_inside_arg)
                  ctf_integer(int, count_total, count_total_arg) ctf_float(float, x, x_arg)
                    ctf_float(float, y, y_arg) ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, monte_carlo_result,
  TP_ARGS(
    const void *, node_handle_arg, const double, pi_estimate_arg, const int, total_iterations_arg,
    const int, count_inside_arg, const int, count_total_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_float(double, pi_estimate, pi_estimate_arg)
                ctf_integer(int, total_iterations, total_iterations_arg)
                  ctf_integer(int, count_inside, count_inside_arg)
                    ctf_integer(int, count_total, count_total_arg)
                      ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, monte_carlo_reset, TP_ARGS(const void *, node_handle_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_string(version, anytime_tracing_VERSION)))

// ==================== YOLO ====================

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, yolo_init,
  TP_ARGS(
    const void *, node_handle_arg, const int, batch_size_arg, const bool, is_reactive_proactive_arg,
    const bool, is_sync_async_arg, const char *, weights_path_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_integer(int, batch_size, batch_size_arg)
                ctf_integer(bool, is_reactive_proactive, is_reactive_proactive_arg)
                  ctf_integer(bool, is_sync_async, is_sync_async_arg)
                    ctf_string(weights_path, weights_path_arg)
                      ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, yolo_layer_start,
  TP_ARGS(const void *, node_handle_arg, const int, layer_num_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_integer(int, layer_num, layer_num_arg)
                ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, yolo_layer_end,
  TP_ARGS(const void *, node_handle_arg, const int, layer_num_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_integer(int, layer_num, layer_num_arg)
                ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, yolo_exit_calculation_start,
  TP_ARGS(const void *, node_handle_arg, const int, layer_num_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_integer(int, layer_num, layer_num_arg)
                ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, yolo_exit_calculation_end,
  TP_ARGS(const void *, node_handle_arg, const int, layer_num_arg, const int, num_detections_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_integer(int, layer_num, layer_num_arg)
                ctf_integer(int, num_detections, num_detections_arg)
                  ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, yolo_detection,
  TP_ARGS(
    const void *, node_handle_arg, const int, layer_num_arg, const int, detection_id_arg,
    const float, confidence_arg, const int, class_id_arg, const float, bbox_x_arg, const float,
    bbox_y_arg, const float, bbox_width_arg, const float, bbox_height_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_integer(int, layer_num, layer_num_arg)
                ctf_integer(int, detection_id, detection_id_arg)
                  ctf_float(float, confidence, confidence_arg)
                    ctf_integer(int, class_id, class_id_arg) ctf_float(float, bbox_x, bbox_x_arg)
                      ctf_float(float, bbox_y, bbox_y_arg)
                        ctf_float(float, bbox_width, bbox_width_arg)
                          ctf_float(float, bbox_height, bbox_height_arg)
                            ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, yolo_result,
  TP_ARGS(
    const void *, node_handle_arg, const int, processed_layers_arg, const int,
    result_processed_layers_arg, const int, total_detections_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_integer(int, processed_layers, processed_layers_arg)
                ctf_integer(int, result_processed_layers, result_processed_layers_arg)
                  ctf_integer(int, total_detections, total_detections_arg)
                    ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, yolo_reset, TP_ARGS(const void *, node_handle_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, yolo_image_processed,
  TP_ARGS(const void *, node_handle_arg, const int, image_width_arg, const int, image_height_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_integer(int, image_width, image_width_arg)
                ctf_integer(int, image_height, image_height_arg)
                  ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, yolo_cuda_callback,
  TP_ARGS(const void *, node_handle_arg, const int, processed_layers_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_integer(int, processed_layers, processed_layers_arg)
                ctf_string(version, anytime_tracing_VERSION)))

// ==================== Interference ====================

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, interference_timer_init,
  TP_ARGS(
    const void *, node_handle_arg, const int, timer_period_ms_arg, const int,
    execution_time_ms_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_integer(int, timer_period_ms, timer_period_ms_arg)
                ctf_integer(int, execution_time_ms, execution_time_ms_arg)
                  ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, interference_timer_callback_entry,
  TP_ARGS(const void *, node_handle_arg, const uint64_t, execution_count_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_integer(uint64_t, execution_count, execution_count_arg)
                ctf_string(version, anytime_tracing_VERSION)))

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER, interference_timer_callback_exit,
  TP_ARGS(
    const void *, node_handle_arg, const uint64_t, execution_count_arg, const int64_t,
    actual_duration_ns_arg),
  TP_FIELDS(ctf_integer_hex(const void *, node_handle, node_handle_arg)
              ctf_integer(uint64_t, execution_count, execution_count_arg)
                ctf_integer(int64_t, actual_duration_ns, actual_duration_ns_arg)
                  ctf_string(version, anytime_tracing_VERSION)))

#endif  // _ANYTIME_TRACING__TP_CALL_H_

#include <lttng/tracepoint-event.h>

#endif  // ANYTIME_TRACING__TP_CALL_H_
