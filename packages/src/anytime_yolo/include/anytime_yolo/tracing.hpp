#ifndef ANYTIME_YOLO_TRACING_HPP
#define ANYTIME_YOLO_TRACING_HPP

#include "anytime_tracing/anytime_tracetools.h"

#include <rclcpp/rclcpp.hpp>

// Helper macros for tracing in anytime_yolo

#define TRACE_YOLO_INIT(node, batch_size, is_reactive_proactive, is_sync_async, weights_path) \
  ANYTIME_TRACEPOINT(                                                                         \
    yolo_init, static_cast<const void *>(node->get_node_base_interface().get()), batch_size,  \
    is_reactive_proactive, is_sync_async, weights_path)

#define TRACE_YOLO_LAYER_START(node, layer_num) \
  ANYTIME_TRACEPOINT(                           \
    yolo_layer_start, static_cast<const void *>(node->get_node_base_interface().get()), layer_num)

#define TRACE_YOLO_LAYER_END(node, layer_num) \
  ANYTIME_TRACEPOINT(                         \
    yolo_layer_end, static_cast<const void *>(node->get_node_base_interface().get()), layer_num)

#define TRACE_YOLO_EXIT_CALCULATION_START(node, layer_num)                                         \
  ANYTIME_TRACEPOINT(                                                                              \
    yolo_exit_calculation_start, static_cast<const void *>(node->get_node_base_interface().get()), \
    layer_num)

#define TRACE_YOLO_EXIT_CALCULATION_END(node, layer_num, num_detections)                         \
  ANYTIME_TRACEPOINT(                                                                            \
    yolo_exit_calculation_end, static_cast<const void *>(node->get_node_base_interface().get()), \
    layer_num, num_detections)

#define TRACE_YOLO_DETECTION(                                                                    \
  node, layer_num, detection_id, confidence, class_id, bbox_x, bbox_y, bbox_width, bbox_height)  \
  ANYTIME_TRACEPOINT(                                                                            \
    yolo_detection, static_cast<const void *>(node->get_node_base_interface().get()), layer_num, \
    detection_id, confidence, class_id, bbox_x, bbox_y, bbox_width, bbox_height)

#define TRACE_YOLO_RESULT(node, processed_layers, result_processed_layers, total_detections) \
  ANYTIME_TRACEPOINT(                                                                        \
    yolo_result, static_cast<const void *>(node->get_node_base_interface().get()),           \
    processed_layers, result_processed_layers, total_detections)

#define TRACE_YOLO_RESET(node) \
  ANYTIME_TRACEPOINT(yolo_reset, static_cast<const void *>(node->get_node_base_interface().get()))

#define TRACE_YOLO_IMAGE_PROCESSED(node, image_width, image_height)                         \
  ANYTIME_TRACEPOINT(                                                                       \
    yolo_image_processed, static_cast<const void *>(node->get_node_base_interface().get()), \
    image_width, image_height)

#define TRACE_YOLO_CUDA_CALLBACK(node, processed_layers)                                  \
  ANYTIME_TRACEPOINT(                                                                     \
    yolo_cuda_callback, static_cast<const void *>(node->get_node_base_interface().get()), \
    processed_layers)

#endif  // ANYTIME_YOLO_TRACING_HPP
