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

#include <algorithm>
#include <atomic>
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

    if constexpr (isSyncAsync) {
      // Async mode: only attach callback when actually submitting a GPU layer
      bool is_layer_submission =
        (yolo_state_->currentStage == InferenceState::LAYER_PROCESSING &&
         yolo_state_->currentIndex < MAX_NETWORK_LAYERS);

      void (*callback)(void *) = is_layer_submission ? forward_finished_callback : nullptr;

      yolo_.inferStep(*yolo_state_, true, callback, this);

      if (is_layer_submission) {
        submitted_layers_++;
      }
    } else {
      // Sync mode: no callback, increment directly
      yolo_.inferStep(*yolo_state_, false, nullptr, nullptr);
      processed_layers_++;
      TRACE_YOLO_LAYER_END(this->node_, processed_layers_);
      RCLCPP_DEBUG(this->node_->get_logger(), "Processed layers: %d", processed_layers_);
      this->send_feedback();
    }
  }

  // Override to limit iterations by remaining layers
  int get_batch_iterations() const override
  {
    if constexpr (isSyncAsync) {
      // Async mode: handle stage-progress transitions
      if (yolo_state_->isCompleted()) {
        return 0;
      }
      int layers_left_to_submit = MAX_NETWORK_LAYERS - submitted_layers_;
      if (layers_left_to_submit > 0) {
        return std::min(this->batch_size_, layers_left_to_submit);
      }
      // All layers submitted but not completed — need EXIT/NMS transitions
      // Return 1 to drive the state machine forward (avoids deadlock)
      return 1;
    } else {
      // Sync mode: original behavior
      int layers_left = MAX_NETWORK_LAYERS - processed_layers_;
      return std::min(this->batch_size_, layers_left);
    }
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

    // Synchronize CUDA stream to drain all in-flight callbacks
    yolo_.synchronize();

    // Reset YOLO state
    yolo_state_->restart(input_cuda_buffer_);

    processed_layers_ = 0;
    submitted_layers_ = 0;
    completion_signals_.store(0, std::memory_order_relaxed);
    result_processed_layers_ = 0;
  }

  bool should_finish() const override { return yolo_state_->isCompleted(); }

  // ---------------- CUDA Callback Function -----------------
  // Minimal signal-only callback — runs on CUDA host thread.
  // NO goal_handle_ access, NO logging, NO state mutation except atomic signal.
  static void CUDART_CB forward_finished_callback(void * userData)
  {
    auto this_ptr = static_cast<AnytimeManagement *>(userData);
    this_ptr->completion_signals_.fetch_add(1, std::memory_order_release);
    this_ptr->notify_waitable();
  }

  // Drain GPU completion signals (runs on executor thread — safe for ROS 2 access)
  void process_gpu_completions() override
  {
    int signals = completion_signals_.exchange(0, std::memory_order_acq_rel);
    if (signals > 0) {
      // Clamp to outstanding submissions to prevent overcounting
      int outstanding = submitted_layers_ - processed_layers_;
      int newly_completed = std::clamp(signals, 0, outstanding);
      processed_layers_ += newly_completed;

      TRACE_YOLO_CUDA_CALLBACK(this->node_, processed_layers_);
      RCLCPP_DEBUG(
        this->node_->get_logger(),
        "GPU completions: %d signals, %d accepted, %d total layers",
        signals, newly_completed, processed_layers_);
    }
  }

protected:
  std::string weights_path_;  // Path to YOLO weights

  AnytimeYOLO yolo_;
  std::unique_ptr<InferenceState> yolo_state_;  // YOLO inference state as pointer
  bool halfPrecision = false;                   // Flag for half precision
  CudaHostBuffer input_cuda_buffer_;            // Input image buffer

  int processed_layers_ = 0;         // Completed GPU layers (executor-thread owned)
  int submitted_layers_ = 0;         // Layers submitted to GPU
  int result_processed_layers_ = 0;  // Processed layers snapshot for result
  std::atomic<int> completion_signals_{0};  // Incremented by CUDA callback, drained by executor
};

#endif  // ANYTIME_MANAGEMENT_HPP
