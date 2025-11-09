#ifndef ANYTIME_MANAGEMENT_HPP
#define ANYTIME_MANAGEMENT_HPP

#include "anytime_core/anytime_base.hpp"
#include "anytime_core/anytime_waitable.hpp"
#include "anytime_interfaces/action/yolo.hpp"
#include "anytime_yolo/tracing.hpp"
#include "anytime_yolo/yolo.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <opencv2/opencv.hpp>
#include <rclcpp/logging.hpp>

#include <cv_bridge/cv_bridge.h>

#include <cstdint>
#include <future>
#include <memory>
#include <random>

// Aliases for better readability
using Anytime = anytime_interfaces::action::Yolo;
using AnytimeGoalHandle = rclcpp_action::ServerGoalHandle<Anytime>;

// Anytime Management class template
template <bool isReactiveProactive, bool isSyncAsync>
class AnytimeManagement : public anytime_core::AnytimeBase<Anytime, AnytimeGoalHandle>
{
public:
  // Constants
  static constexpr int MAX_NETWORK_LAYERS = 25;
  static constexpr int IMAGE_SIZE = 640;

  // Constructor
  AnytimeManagement(rclcpp::Node * node, int batch_size = 1, const std::string & weights_path = "")
  : weights_path_(weights_path),
    yolo_(weights_path, false),
    yolo_state_(std::make_unique<InferenceState>(yolo_.createInferenceState())),
    input_cuda_buffer_(
      IMAGE_SIZE * IMAGE_SIZE * 3 * (halfPrecision ? sizeof(__half) : sizeof(float)))
  {
    // Initialize common base class functionality
    this->template initialize_anytime_base<isReactiveProactive>(node, batch_size);

    TRACE_YOLO_INIT(node, batch_size, isReactiveProactive, isSyncAsync, weights_path.c_str());
  }

  // ----------------- Domain-Specific Implementations -----------------

  void compute_single_iteration() override
  {
    RCLCPP_DEBUG(this->node_->get_logger(), "YOLO compute single iteration called");
    TRACE_YOLO_LAYER_START(this->node_, processed_layers_);

    void (*callback)(void *) = nullptr;
    if constexpr (isSyncAsync) {
      callback = forward_finished_callback;
    }

    RCLCPP_DEBUG(this->node_->get_logger(), "Callback function is null: %d", callback == nullptr);
    RCLCPP_DEBUG(this->node_->get_logger(), "Calling inferStep");

    yolo_.inferStep(*yolo_state_, isSyncAsync, callback, this);

    RCLCPP_DEBUG(this->node_->get_logger(), "Finished single iteration");

    if constexpr (!isSyncAsync) {
      // Increment processed layers counter for sync mode
      processed_layers_++;
      TRACE_YOLO_LAYER_END(this->node_, processed_layers_);
      RCLCPP_DEBUG(this->node_->get_logger(), "Processed layers: %d", processed_layers_);
      this->send_feedback();
    }
    // Async mode: callback handles layer counting and feedback
  }

  // Override to limit iterations by remaining layers
  int get_batch_iterations() const override
  {
    constexpr int MAX_NETWORK_LAYERS = 25;
    int max_layers = MAX_NETWORK_LAYERS;
    int layers_left = max_layers - processed_layers_;
    int iterations = std::min(this->batch_size_, layers_left);
    return iterations;
  }

  void populate_feedback(std::shared_ptr<Anytime::Feedback> feedback) override
  {
    feedback->processed_layers = processed_layers_;
    for (const auto & detection : this->result_->detections) {
      RCLCPP_DEBUG(
        this->node_->get_logger(), "Adding detection to feedback, class ID: %s, score: %f",
        detection.results[0].hypothesis.class_id.c_str(), detection.results[0].hypothesis.score);
      feedback->detections.push_back(detection);
    }
    RCLCPP_DEBUG(
      this->node_->get_logger(), "Feedback populated, processed layers: %d", processed_layers_);
  }

  void populate_result(std::shared_ptr<Anytime::Result> result) override
  {
    RCLCPP_DEBUG(this->node_->get_logger(), "YOLO result populated");

    result_processed_layers_ = processed_layers_;
    std::vector<float> yolo_result;

    // Trace exit calculation start
    TRACE_YOLO_EXIT_CALCULATION_START(this->node_, processed_layers_);

    if constexpr (isReactiveProactive) {
      RCLCPP_DEBUG(this->node_->get_logger(), "Calculating latest exit");
      yolo_result = yolo_.calculateLatestExit(*yolo_state_);
    } else {
      RCLCPP_DEBUG(this->node_->get_logger(), "Finishing early");
      yolo_result = yolo_.finishEarly(*yolo_state_);
    }

    // Count valid detections (confidence > 0)
    int detection_count = 0;
    for (size_t i = 0; i < yolo_result.size(); i += 6) {
      if (i + 4 < yolo_result.size() && yolo_result[i + 4] > 0.0) {
        detection_count++;
      }
    }

    // Trace exit calculation end with detection count
    TRACE_YOLO_EXIT_CALCULATION_END(this->node_, processed_layers_, detection_count);

    for (size_t i = 0; i < yolo_result.size(); i += 6) {
      // skip if confidence is 0.0
      if (yolo_result[i + 4] == 0.0) {
        continue;
      }

      if (i + 5 >= yolo_result.size()) break;  // Safety check

      // Create a new detection
      vision_msgs::msg::Detection2D detection;

      // Set the bounding box
      detection.bbox.center.position.x = (yolo_result[i] + yolo_result[i + 2]) / 2;
      detection.bbox.center.position.y = (yolo_result[i + 1] + yolo_result[i + 3]) / 2;
      detection.bbox.size_x = yolo_result[i + 2] - yolo_result[i];
      detection.bbox.size_y = yolo_result[i + 3] - yolo_result[i + 1];

      // Get original image dimensions
      const auto & image_msg = this->goal_handle_->get_goal()->image;
      float orig_width = static_cast<float>(image_msg.width);
      float orig_height = static_cast<float>(image_msg.height);

      // Calculate scaling factors
      float scale_x = orig_width / static_cast<float>(IMAGE_SIZE);
      float scale_y = orig_height / static_cast<float>(IMAGE_SIZE);

      // Adjust bounding box to original image size
      detection.bbox.center.position.x *= scale_x;
      detection.bbox.center.position.y *= scale_y;
      detection.bbox.size_x *= scale_x;
      detection.bbox.size_y *= scale_y;

      // Set the confidence
      detection.results.resize(1);
      detection.results[0].hypothesis.score = yolo_result[i + 4];

      // Set the class ID
      detection.results[0].hypothesis.class_id =
        std::to_string(static_cast<int>(yolo_result[i + 5]));

      // Add the detection to the result
      result->detections.push_back(detection);

      // Trace individual detection with original (pre-scaled) bounding box coordinates
      TRACE_YOLO_DETECTION(
        this->node_, processed_layers_, static_cast<int>(i / 6), yolo_result[i + 4],
        static_cast<int>(yolo_result[i + 5]), yolo_result[i], yolo_result[i + 1],
        yolo_result[i + 2] - yolo_result[i], yolo_result[i + 3] - yolo_result[i + 1]);
    }

    // Add additional information to result
    result->average_batch_time = this->average_computation_time_;
    result->batch_size = this->batch_size_;
    result->processed_layers = processed_layers_;
    result->result_processed_layers = result_processed_layers_;

    TRACE_YOLO_RESULT(
      this->node_, processed_layers_, result_processed_layers_, result->detections.size());
  }

  void reset_domain_state() override
  {
    TRACE_YOLO_RESET(this->node_);
    RCLCPP_DEBUG(this->node_->get_logger(), "YOLO reset domain state called");

    // Process image data from the goal handle
    if (this->goal_handle_) {
      const auto & goal = this->goal_handle_->get_goal();
      const auto & image_msg = goal->image;

      // Convert ROS Image message to OpenCV image
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
      } catch (cv_bridge::Exception & e) {
        RCLCPP_ERROR(this->node_->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

      // Get the OpenCV image
      cv::Mat img = cv_ptr->image;

      TRACE_YOLO_IMAGE_PROCESSED(this->node_, img.cols, img.rows);

      cv::Mat blob = cv::dnn::blobFromImage(
        img,                               // input image
        1.0 / 255.0,                       // scale factor (normalization)
        cv::Size(IMAGE_SIZE, IMAGE_SIZE),  // output size
        cv::Scalar(0, 0, 0),               // mean subtraction (none here)
        true,                              // swapRB - converts BGR to RGB
        false                              // crop - no cropping
      );

      if (halfPrecision) {
        blob.convertTo(blob, CV_16F);  // Convert to half precision
      } else {
        blob.convertTo(blob, CV_32F);  // Convert to float
      }

      const size_t data_size = blob.total() * blob.elemSize();

      // check if the buffer is large enough
      if (data_size > input_cuda_buffer_.size) {
        RCLCPP_ERROR(
          this->node_->get_logger(), "Buffer size is not large enough: %zu > %zu", data_size,
          input_cuda_buffer_.size);
        throw std::runtime_error("Buffer size is not large enough");
      }
      // Copy data to the buffer
      if (!input_cuda_buffer_.copyFromHost(blob.data, data_size)) {
        RCLCPP_ERROR(this->node_->get_logger(), "Error copying data to buffer");
        throw std::runtime_error("Error copying data to buffer");
      }
    }

    // Reset YOLO state
    yolo_state_->restart(input_cuda_buffer_);

    processed_layers_ = 0;
    result_processed_layers_ = 0;
  }

  bool should_finish() const override { return yolo_state_->isCompleted(); }

  // Return GPU callback for async mode, nullptr for sync mode
  void * get_iteration_callback() override
  {
    if constexpr (isSyncAsync) {
      return reinterpret_cast<void *>(&forward_finished_callback);
    } else {
      return nullptr;
    }
  }

  // ---------------- CUDA Callback Function -----------------

  static void CUDART_CB forward_finished_callback(void * userData)
  {
    auto this_ptr = static_cast<AnytimeManagement *>(userData);

    std::string goal_id_str = this_ptr->goal_handle_
                                ? rclcpp_action::to_string(this_ptr->goal_handle_->get_goal_id())
                                : "null";

    RCLCPP_DEBUG(
      this_ptr->node_->get_logger(), "[Goal ID: %s] Forward finished callback",
      goal_id_str.c_str());

    // Check if we should stop processing
    if (
      this_ptr->goal_handle_->is_canceling() || !this_ptr->goal_handle_->is_executing() ||
      !this_ptr->is_running()) {
      RCLCPP_DEBUG(
        this_ptr->node_->get_logger(),
        "[Goal ID: %s] Goal handle is canceling, stopping computation", goal_id_str.c_str());
      return;
    }

    // Increment processed layers counter for async mode
    this_ptr->processed_layers_++;
    TRACE_YOLO_CUDA_CALLBACK(this_ptr->node_, this_ptr->processed_layers_);
    RCLCPP_DEBUG(
      this_ptr->node_->get_logger(), "[Goal ID: %s] Processed layers: %d", goal_id_str.c_str(),
      this_ptr->processed_layers_);

    // Trigger the waitable when batch is complete or when reaching/exceeding max layers
    // This allows the main anytime function to continue with the next iteration
    if (
      (this_ptr->processed_layers_ % this_ptr->batch_size_ == 0) ||
      (this_ptr->processed_layers_ >= MAX_NETWORK_LAYERS)) {
      RCLCPP_DEBUG(
        this_ptr->node_->get_logger(),
        "[Goal ID: %s] Batch complete (%d layers processed), triggering waitable",
        goal_id_str.c_str(), this_ptr->processed_layers_);
      this_ptr->notify_waitable();
    }
  }

protected:
  std::string weights_path_;  // Path to YOLO weights

  AnytimeYOLO yolo_;
  std::unique_ptr<InferenceState> yolo_state_;  // YOLO inference state as pointer
  bool halfPrecision = false;                   // Flag for half precision
  CudaHostBuffer input_cuda_buffer_;            // Input image buffer

  int processed_layers_ = 0;         // Counter for processed network layers
  int result_processed_layers_ = 0;  // Counter for processed network layers in result
};

#endif  // ANYTIME_MANAGEMENT_HPP
