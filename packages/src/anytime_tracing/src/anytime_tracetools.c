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

#include "anytime_tracing/anytime_tracetools.h"

#ifndef ANYTIME_TRACING_DISABLED

#ifdef ANYTIME_TRACING_LTTNG_ENABLED
# include "anytime_tracing/tp_call.h"
# define CONDITIONAL_TP(...) \
  tracepoint(TRACEPOINT_PROVIDER, __VA_ARGS__)
#else
# define CONDITIONAL_TP(...)
#endif

bool anytime_trace_compile_status()
{
#ifdef ANYTIME_TRACING_LTTNG_ENABLED
  return true;
#else
  return false;
#endif
}

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#else
# pragma warning(push)
# pragma warning(disable: 4100)
#endif

// ==================== Core: AnytimeBase ====================

void ANYTIME_TRACEPOINT(
  anytime_base_init,
  const void * node_handle,
  const int batch_size,
  const bool is_reactive_proactive)
{
  CONDITIONAL_TP(anytime_base_init, node_handle, batch_size, is_reactive_proactive);
}

void ANYTIME_TRACEPOINT(anytime_base_activate, const void * node_handle)
{
  CONDITIONAL_TP(anytime_base_activate, node_handle);
}

void ANYTIME_TRACEPOINT(anytime_base_deactivate, const void * node_handle)
{
  CONDITIONAL_TP(anytime_base_deactivate, node_handle);
}

void ANYTIME_TRACEPOINT(anytime_base_reset, const void * node_handle)
{
  CONDITIONAL_TP(anytime_base_reset, node_handle);
}

void ANYTIME_TRACEPOINT(reactive_anytime_function_entry, const void * node_handle)
{
  CONDITIONAL_TP(reactive_anytime_function_entry, node_handle);
}

void ANYTIME_TRACEPOINT(
  reactive_anytime_function_exit,
  const void * node_handle,
  const bool should_finish,
  const bool should_cancel)
{
  CONDITIONAL_TP(reactive_anytime_function_exit, node_handle, should_finish, should_cancel);
}

void ANYTIME_TRACEPOINT(proactive_anytime_function_entry, const void * node_handle)
{
  CONDITIONAL_TP(proactive_anytime_function_entry, node_handle);
}

void ANYTIME_TRACEPOINT(
  proactive_anytime_function_exit,
  const void * node_handle,
  const bool should_finish,
  const bool should_cancel)
{
  CONDITIONAL_TP(proactive_anytime_function_exit, node_handle, should_finish, should_cancel);
}

void ANYTIME_TRACEPOINT(
  anytime_compute_entry,
  const void * node_handle,
  const int batch_size)
{
  CONDITIONAL_TP(anytime_compute_entry, node_handle, batch_size);
}

void ANYTIME_TRACEPOINT(
  anytime_compute_exit,
  const void * node_handle,
  const int iterations_completed,
  const int64_t computation_time_ns,
  const int64_t average_time_ns)
{
  CONDITIONAL_TP(
    anytime_compute_exit, node_handle, iterations_completed, computation_time_ns, average_time_ns);
}

void ANYTIME_TRACEPOINT(
  anytime_compute_iteration,
  const void * node_handle,
  const int iteration_num)
{
  CONDITIONAL_TP(anytime_compute_iteration, node_handle, iteration_num);
}

void ANYTIME_TRACEPOINT(anytime_send_feedback_entry, const void * node_handle)
{
  CONDITIONAL_TP(anytime_send_feedback_entry, node_handle);
}

void ANYTIME_TRACEPOINT(anytime_send_feedback_exit, const void * node_handle)
{
  CONDITIONAL_TP(anytime_send_feedback_exit, node_handle);
}

void ANYTIME_TRACEPOINT(anytime_calculate_result_entry, const void * node_handle)
{
  CONDITIONAL_TP(anytime_calculate_result_entry, node_handle);
}

void ANYTIME_TRACEPOINT(anytime_calculate_result_exit, const void * node_handle)
{
  CONDITIONAL_TP(anytime_calculate_result_exit, node_handle);
}

// ==================== Core: AnytimeServer ====================

void ANYTIME_TRACEPOINT(
  anytime_server_init,
  const void * node_handle,
  const char * node_name)
{
  CONDITIONAL_TP(anytime_server_init, node_handle, node_name);
}

void ANYTIME_TRACEPOINT(
  anytime_server_handle_goal,
  const void * node_handle,
  const bool accepted)
{
  CONDITIONAL_TP(anytime_server_handle_goal, node_handle, accepted);
}

void ANYTIME_TRACEPOINT(anytime_server_handle_cancel, const void * node_handle)
{
  CONDITIONAL_TP(anytime_server_handle_cancel, node_handle);
}

void ANYTIME_TRACEPOINT(anytime_server_handle_accepted, const void * node_handle)
{
  CONDITIONAL_TP(anytime_server_handle_accepted, node_handle);
}

// ==================== Core: AnytimeClient ====================

void ANYTIME_TRACEPOINT(
  anytime_client_init,
  const void * node_handle,
  const char * node_name)
{
  CONDITIONAL_TP(anytime_client_init, node_handle, node_name);
}

void ANYTIME_TRACEPOINT(anytime_client_send_goal, const void * node_handle)
{
  CONDITIONAL_TP(anytime_client_send_goal, node_handle);
}

void ANYTIME_TRACEPOINT(
  anytime_client_goal_response,
  const void * node_handle,
  const bool accepted)
{
  CONDITIONAL_TP(anytime_client_goal_response, node_handle, accepted);
}

void ANYTIME_TRACEPOINT(anytime_client_feedback, const void * node_handle)
{
  CONDITIONAL_TP(anytime_client_feedback, node_handle);
}

void ANYTIME_TRACEPOINT(
  anytime_client_result,
  const void * node_handle,
  const int result_code)
{
  CONDITIONAL_TP(anytime_client_result, node_handle, result_code);
}

void ANYTIME_TRACEPOINT(anytime_client_cancel_request, const void * node_handle)
{
  CONDITIONAL_TP(anytime_client_cancel_request, node_handle);
}

void ANYTIME_TRACEPOINT(
  anytime_client_cancel_response,
  const void * node_handle,
  const bool accepted)
{
  CONDITIONAL_TP(anytime_client_cancel_response, node_handle, accepted);
}

void ANYTIME_TRACEPOINT(
  anytime_client_goal_sent,
  const void * node_handle,
  const int64_t timestamp_ns)
{
  CONDITIONAL_TP(anytime_client_goal_sent, node_handle, timestamp_ns);
}

void ANYTIME_TRACEPOINT(
  anytime_client_cancel_sent,
  const void * node_handle,
  const int64_t timestamp_ns)
{
  CONDITIONAL_TP(anytime_client_cancel_sent, node_handle, timestamp_ns);
}

void ANYTIME_TRACEPOINT(
  anytime_client_goal_finished,
  const void * node_handle,
  const int64_t timestamp_ns,
  const int result_code)
{
  CONDITIONAL_TP(anytime_client_goal_finished, node_handle, timestamp_ns, result_code);
}

// ==================== Monte Carlo ====================

void ANYTIME_TRACEPOINT(
  monte_carlo_init,
  const void * node_handle,
  const int batch_size,
  const bool is_reactive_proactive)
{
  CONDITIONAL_TP(monte_carlo_init, node_handle, batch_size, is_reactive_proactive);
}

void ANYTIME_TRACEPOINT(
  monte_carlo_iteration,
  const void * node_handle,
  const int iteration_num,
  const int count_inside,
  const int count_total,
  const float x,
  const float y)
{
  CONDITIONAL_TP(monte_carlo_iteration, node_handle, iteration_num, count_inside, count_total, x, y);
}

void ANYTIME_TRACEPOINT(
  monte_carlo_result,
  const void * node_handle,
  const double pi_estimate,
  const int total_iterations,
  const int count_inside,
  const int count_total)
{
  CONDITIONAL_TP(monte_carlo_result, node_handle, pi_estimate, total_iterations, count_inside, count_total);
}

void ANYTIME_TRACEPOINT(monte_carlo_reset, const void * node_handle)
{
  CONDITIONAL_TP(monte_carlo_reset, node_handle);
}

// ==================== YOLO ====================

void ANYTIME_TRACEPOINT(
  yolo_init,
  const void * node_handle,
  const int batch_size,
  const bool is_reactive_proactive,
  const bool is_sync_async,
  const char * weights_path)
{
  CONDITIONAL_TP(yolo_init, node_handle, batch_size, is_reactive_proactive, is_sync_async, weights_path);
}

void ANYTIME_TRACEPOINT(
  yolo_layer_start,
  const void * node_handle,
  const int layer_num)
{
  CONDITIONAL_TP(yolo_layer_start, node_handle, layer_num);
}

void ANYTIME_TRACEPOINT(
  yolo_layer_end,
  const void * node_handle,
  const int layer_num)
{
  CONDITIONAL_TP(yolo_layer_end, node_handle, layer_num);
}

void ANYTIME_TRACEPOINT(
  yolo_exit_calculation_start,
  const void * node_handle,
  const int layer_num)
{
  CONDITIONAL_TP(yolo_exit_calculation_start, node_handle, layer_num);
}

void ANYTIME_TRACEPOINT(
  yolo_exit_calculation_end,
  const void * node_handle,
  const int layer_num,
  const int num_detections)
{
  CONDITIONAL_TP(yolo_exit_calculation_end, node_handle, layer_num, num_detections);
}

void ANYTIME_TRACEPOINT(
  yolo_detection,
  const void * node_handle,
  const int layer_num,
  const int detection_id,
  const float confidence,
  const int class_id,
  const float bbox_x,
  const float bbox_y,
  const float bbox_width,
  const float bbox_height)
{
  CONDITIONAL_TP(
    yolo_detection, node_handle, layer_num, detection_id, confidence, class_id,
    bbox_x, bbox_y, bbox_width, bbox_height);
}

void ANYTIME_TRACEPOINT(
  yolo_result,
  const void * node_handle,
  const int processed_layers,
  const int result_processed_layers,
  const int total_detections)
{
  CONDITIONAL_TP(yolo_result, node_handle, processed_layers, result_processed_layers, total_detections);
}

void ANYTIME_TRACEPOINT(yolo_reset, const void * node_handle)
{
  CONDITIONAL_TP(yolo_reset, node_handle);
}

void ANYTIME_TRACEPOINT(
  yolo_image_processed,
  const void * node_handle,
  const int image_width,
  const int image_height)
{
  CONDITIONAL_TP(yolo_image_processed, node_handle, image_width, image_height);
}

void ANYTIME_TRACEPOINT(
  yolo_cuda_callback,
  const void * node_handle,
  const int processed_layers)
{
  CONDITIONAL_TP(yolo_cuda_callback, node_handle, processed_layers);
}

// ==================== Interference ====================

void ANYTIME_TRACEPOINT(
  interference_timer_init,
  const void * node_handle,
  const int timer_period_ms,
  const int execution_time_ms)
{
  CONDITIONAL_TP(interference_timer_init, node_handle, timer_period_ms, execution_time_ms);
}

void ANYTIME_TRACEPOINT(
  interference_timer_callback_entry,
  const void * node_handle,
  const uint64_t execution_count)
{
  CONDITIONAL_TP(interference_timer_callback_entry, node_handle, execution_count);
}

void ANYTIME_TRACEPOINT(
  interference_timer_callback_exit,
  const void * node_handle,
  const uint64_t execution_count,
  const int64_t actual_duration_ns)
{
  CONDITIONAL_TP(interference_timer_callback_exit, node_handle, execution_count, actual_duration_ns);
}

#ifndef _WIN32
# pragma GCC diagnostic pop
#else
# pragma warning(pop)
#endif

#endif  // ANYTIME_TRACING_DISABLED
