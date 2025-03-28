#ifndef ANYTIME_MANAGEMENT_HPP
#define ANYTIME_MANAGEMENT_HPP

#include "anytime_yolo/anytime_base.hpp"
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
template <bool isReactiveProactive, bool isSingleMulti, bool isPassiveCooperative, bool isSyncAsync>
class AnytimeManagement : public AnytimeBase<double, Anytime, AnytimeGoalHandle>
{
public:
  // Constructor
  AnytimeManagement(rclcpp::Node * node, int batch_size = 1, const std::string & weights_path = "")
  : node_(node), batch_size_(batch_size), weights_path_(weights_path), yolo_(weights_path, false)
  {
    // callback group
    compute_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // --- Proactive Variables ---
    if constexpr (isReactiveProactive) {
      anytime_iteration_waitable_ =
        std::make_shared<AnytimeWaitable>([this]() { this->proactive_function(); });
      anytime_check_finish_waitable_ =
        std::make_shared<AnytimeWaitable>([this]() { this->check_cancel_and_finish_proactive(); });
      node_->get_node_waitables_interface()->add_waitable(
        anytime_iteration_waitable_, compute_callback_group_);
      node_->get_node_waitables_interface()->add_waitable(
        anytime_check_finish_waitable_,
        node_->get_node_base_interface()->get_default_callback_group());
    }
    // --- Reactive Variables ---
    else {
      anytime_iteration_waitable_ =
        std::make_shared<AnytimeWaitable>([this]() { this->reactive_function(); });
      node_->get_node_waitables_interface()->add_waitable(
        anytime_iteration_waitable_, compute_callback_group_);
    }
  }

  // ----------------- Reactive Functions -----------------

  void reactive_function()
  {
    auto feedback = std::make_shared<Anytime::Feedback>();
    feedback->processed_layers = processed_layers_;
    this->goal_handle_->publish_feedback(feedback);
    if (check_cancel_and_finish_reactive()) {
      return;
    } else {
      compute();
    }
  }

  bool check_cancel_and_finish_reactive() override
  {
    bool should_finish = yolo_state_->isCompleted();
    bool should_cancel = this->goal_handle_->is_canceling();

    if (should_finish || should_cancel) {
      this->result_->action_server_send_result = this->node_->now();
      this->result_->average_batch_time = this->average_computation_time_;
      this->calculate_result();
      this->result_->action_server_cancel = this->node_->now();

      if (should_cancel) {
        this->goal_handle_->canceled(this->result_);
      } else {
        this->goal_handle_->succeed(this->result_);
      }

      this->deactivate();
      return true;
    }

    return false;
  }

  // ----------------- Proactive Functions -----------------

  // proactive function to approximate Pi
  void proactive_function() { compute(); }

  void check_cancel_and_finish_proactive() override
  {
    calculate_result();
    // Print number of detected objects and processed layers
    RCLCPP_INFO(
      node_->get_logger(), "Detected objects: %zu, Processed layers: %d",
      this->result_->detections.size(), processed_layers_);
    bool should_finish = yolo_state_->isCompleted();
    bool should_cancel = this->goal_handle_->is_canceling() || !this->goal_handle_->is_executing();

    if ((should_finish || should_cancel) && this->is_running()) {
      this->result_->action_server_send_result = this->node_->now();

      this->result_->average_batch_time = average_computation_time_;
      this->result_->action_server_cancel = this->node_->now();

      if (should_cancel) {
        this->goal_handle_->canceled(this->result_);
      } else {
        this->goal_handle_->succeed(this->result_);
      }

      this->deactivate();
      return;
    } else if (!this->is_running()) {
      return;
    }
    notify_iteration();
  }

  // ----------------- Common Functions -----------------

  void start() override { notify_iteration(); }

  // Change to static method for CUDA callback compatibility
  static void CUDART_CB forward_finished_callback(void * userData)
  {
    RCLCPP_INFO(
      static_cast<AnytimeManagement *>(userData)->node_->get_logger(), "Forward finished");

    auto this_ptr = static_cast<AnytimeManagement *>(userData);

    if constexpr (isSyncAsync) {
      // Increment processed layers counter for async mode
      this_ptr->processed_layers_++;
      RCLCPP_INFO(
        this_ptr->node_->get_logger(), "Processed layers: %d", this_ptr->processed_layers_);
    } else if constexpr (!isSyncAsync) {
      // nothing to do for sync mode
    }

    // Notify the waitable
    if constexpr (isReactiveProactive) {
      this_ptr->notify_check_finish();
    } else if constexpr (!isReactiveProactive) {
      this_ptr->notify_iteration();
    }
  }

  void compute() override
  {
    RCLCPP_INFO(node_->get_logger(), "Computing");

    // Start timing
    auto start_time = this->node_->now();

    // Determine synchronicity before the loop
    constexpr bool is_sync_async = isSyncAsync;

    // void (*callback)(void *) = isPassiveCooperative ? forward_finished_callback : nullptr;

    for (int i = 0; i < batch_size_; i++) {
      RCLCPP_INFO(node_->get_logger(), "Computing batch part %d", i);

      void (*callback)(void *);
      // // only set forward_finished_callback for the last batch
      // if (i == batch_size_ - 1) {
      //   callback = forward_finished_callback;
      // } else {
      //   callback = nullptr;
      // }

      callback = forward_finished_callback;

      yolo_.inferStep(*yolo_state_, is_sync_async, callback, this);

      RCLCPP_INFO(node_->get_logger(), "Finished batch part %d", i);

      if constexpr (!isSyncAsync) {
        // Increment processed layers counter for sync mode
        processed_layers_++;
        RCLCPP_INFO(node_->get_logger(), "Processed layers: %d", processed_layers_);
      } else if constexpr (isSyncAsync) {
        // nothing to do for async mode
      }
    }
    RCLCPP_INFO(node_->get_logger(), "Finished computing");

    // End timing
    auto end_time = this->node_->now();
    // Calculate computation time for this batch
    rclcpp::Duration computation_time = end_time - start_time;
    RCLCPP_INFO(
      node_->get_logger(), "Batch computation time: %f ms", computation_time.nanoseconds() / 1e6);

    // Update the average computation time
    batch_count_++;
    if (batch_count_ == 1) {
      average_computation_time_ = computation_time;
    } else {
      average_computation_time_ = rclcpp::Duration(std::chrono::nanoseconds(
        average_computation_time_.nanoseconds() +
        (computation_time.nanoseconds() - average_computation_time_.nanoseconds()) / batch_count_));
    }
  }

  void calculate_result() override
  {
    this->result_ = std::make_shared<Anytime::Result>();
    RCLCPP_INFO(node_->get_logger(), "Calculating result");
    std::vector<float> yolo_result;
    if constexpr (isReactiveProactive) {
      yolo_result = yolo_.calculateLatestExit(*yolo_state_);
    } else if constexpr (!isReactiveProactive) {
      yolo_result = yolo_.finishEarly(*yolo_state_);
    }

    // Convert the YOLO result vector to Detection2D messages
    // The vector format is [x(top left), y(top left), x(bottom right), y(bottom right), width,
    // height, confidence, class_id] repeated for each detection
    this->result_->detections.clear();

    for (size_t i = 0; i < yolo_result.size(); i += 6) {
      // skip if confidence is 0.0
      if (yolo_result[i + 4] == 0.0) {
        continue;
      }

      if (i + 5 >= yolo_result.size()) break;  // Safety check to avoid out-of-bounds access

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
      float scale_x = orig_width / 640.0f;
      float scale_y = orig_height / 640.0f;

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
      this->result_->detections.push_back(detection);
    }

    // Add additional information to result
    this->result_->average_batch_time = average_computation_time_;
    this->result_->batch_size = batch_size_;
    this->result_->processed_layers = processed_layers_;
  }

  // Cancel function
  void notify_cancel() override
  {
    this->result_->action_server_cancel = this->node_->now();
    if constexpr (isReactiveProactive) {
      notify_check_finish();
    } else if constexpr (!isReactiveProactive) {
      // nothing to do for reactive mode
    }
  }

  // Reset function
  void reset() override
  {
    this->result_ = std::make_shared<Anytime::Result>();

    input_cuda_buffer_.free();

    // Process image data from the goal handle
    if (this->goal_handle_) {
      const auto & goal = this->goal_handle_->get_goal();

      // Get the image from the goal
      const auto & image_msg = goal->image;

      // Convert ROS Image message to OpenCV image
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
      } catch (cv_bridge::Exception & e) {
        RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

      // Get the OpenCV image
      cv::Mat img = cv_ptr->image;

      // Resize to 640x640
      cv::Mat resized_img;
      cv::resize(img, resized_img, cv::Size(640, 640));

      // Convert BGR to RGB
      cv::cvtColor(resized_img, resized_img, cv::COLOR_BGR2RGB);

      // Normalize to [0,1]
      cv::Mat float_img;
      resized_img.convertTo(float_img, CV_32FC3, 1.0f / 255.0f);

      // Convert to NCHW format
      std::vector<float> nchw_data;
      const int channels = 3;
      const int height = 640;
      const int width = 640;
      nchw_data.resize(channels * height * width);

      for (int c = 0; c < channels; ++c) {
        for (int h = 0; h < height; ++h) {
          for (int w = 0; w < width; ++w) {
            nchw_data[c * height * width + h * width + w] = float_img.at<cv::Vec3f>(h, w)[c];
          }
        }
      }

      // Allocate CUDA buffer and copy data
      const size_t data_size = nchw_data.size() * sizeof(float);
      if (!input_cuda_buffer_.allocate(data_size)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to allocate CUDA buffer");
        return;
      }

      if (!input_cuda_buffer_.copyFromHost(nchw_data.data(), data_size)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to copy data to CUDA buffer");
        return;
      }
    }

    // Reset YOLO state
    yolo_state_ = std::make_unique<InferenceState>(yolo_.createInferenceState(input_cuda_buffer_));

    this->result_->action_server_receive = this->server_goal_receive_time_;
    this->result_->action_server_accept = this->server_goal_accept_time_;
    this->result_->action_server_start = this->server_goal_start_time_;

    batch_count_ = 0;
    processed_layers_ = 0;  // Reset processed layers counter
    average_computation_time_ = rclcpp::Duration(0, 0);
  }

protected:
  rclcpp::Node * node_;       // Node reference for logging
  int batch_size_;            // Batch size for compute iterations
  std::string weights_path_;  // Path to YOLO weights

  CudaBuffer input_cuda_buffer_;                // Input image buffer
  std::unique_ptr<InferenceState> yolo_state_;  // YOLO inference state as pointer
  AnytimeYOLO yolo_;

  // Batch count and average computation time
  int batch_count_ = 0;
  int processed_layers_ = 0;                         // Counter for processed network layers
  rclcpp::Duration average_computation_time_{0, 0};  // in milliseconds
};

#endif  // ANYTIME_MANAGEMENT_HPP