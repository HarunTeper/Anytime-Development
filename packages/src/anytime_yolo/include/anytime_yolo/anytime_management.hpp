#ifndef ANYTIME_MANAGEMENT_HPP
#define ANYTIME_MANAGEMENT_HPP

#include "anytime_interfaces/action/yolo.hpp"
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
template <bool isReactiveProactive, bool isPassiveCooperative, bool isSyncAsync>
class AnytimeManagement : public AnytimeBase<double, Anytime, AnytimeGoalHandle>
{
public:
  // Constructor
  AnytimeManagement(rclcpp::Node * node, int batch_size = 1, const std::string & weights_path = "")
  : node_(node),
    batch_size_(batch_size),
    weights_path_(weights_path),
    yolo_(weights_path, false),
    yolo_state_(std::make_unique<InferenceState>(yolo_.createInferenceState())),
    input_cuda_buffer_(640 * 640 * 3 * (halfPrecision ? sizeof(__half) : sizeof(float)))
  {
    // callback group
    compute_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // --- Proactive Variables ---
    if constexpr (isReactiveProactive) {
      anytime_iteration_waitable_ =
        std::make_shared<AnytimeWaitable>([this]() { this->proactive_function(); });
      anytime_result_waitable_ =
        std::make_shared<AnytimeWaitable>([this]() { this->proactive_result_function(); });
      anytime_check_finish_waitable_ =
        std::make_shared<AnytimeWaitable>([this]() { this->check_cancel_and_finish_proactive(); });
      node_->get_node_waitables_interface()->add_waitable(
        anytime_iteration_waitable_, compute_callback_group_);
      node_->get_node_waitables_interface()->add_waitable(
        anytime_result_waitable_, compute_callback_group_);
      node_->get_node_waitables_interface()->add_waitable(
        anytime_check_finish_waitable_,
        node_->get_node_base_interface()->get_default_callback_group());
    }
    // --- Reactive Variables ---
    else {
      anytime_iteration_waitable_ =
        std::make_shared<AnytimeWaitable>([this]() { this->reactive_function(); });
      anytime_result_waitable_ =
        std::make_shared<AnytimeWaitable>([this]() { this->reactive_result_function(); });
      anytime_check_finish_waitable_ =
        std::make_shared<AnytimeWaitable>([this]() { this->check_cancel_and_finish_reactive(); });

      node_->get_node_waitables_interface()->add_waitable(
        anytime_iteration_waitable_, compute_callback_group_);
      node_->get_node_waitables_interface()->add_waitable(
        anytime_result_waitable_, compute_callback_group_);
      node_->get_node_waitables_interface()->add_waitable(
        anytime_check_finish_waitable_,
        node_->get_node_base_interface()->get_default_callback_group());
    }
  }

  // ----------------- Reactive Functions -----------------

  void reactive_function() override
  {
    RCLCPP_INFO(node_->get_logger(), "Reactive function called");
    compute();
    if constexpr (!isSyncAsync) {
      notify_result();
    }
  }

  void reactive_result_function() override
  {
    RCLCPP_INFO(node_->get_logger(), "Reactive result function called");
    send_feedback();
    notify_check_finish();
  }

  void check_cancel_and_finish_reactive() override
  {
    RCLCPP_INFO(node_->get_logger(), "Check cancel and finish reactive function called");
    bool should_finish = yolo_state_->isCompleted();
    bool should_cancel = this->goal_handle_->is_canceling();

    if ((should_finish || should_cancel) && this->is_running()) {
      this->calculate_result();

      this->result_->action_server_cancel = this->server_goal_cancel_time_;
      this->result_->action_server_send_result = this->node_->now();
      if (should_cancel) {
        this->goal_handle_->canceled(this->result_);
      } else {
        this->goal_handle_->succeed(this->result_);
      }

      this->deactivate();
      RCLCPP_INFO(
        node_->get_logger(), "Reactive function finished, should finish: %d, should cancel: %d",
        should_finish, should_cancel);
    } else if (!this->is_running()) {
      RCLCPP_INFO(node_->get_logger(), "Reactive function finished previously");
    } else {
      notify_iteration();
    }
  }

  // ----------------- Proactive Functions -----------------

  // proactive function to approximate Pi
  void proactive_function() override
  {
    RCLCPP_INFO(node_->get_logger(), "Proactive function called");
    compute();
    if constexpr (!isSyncAsync) {
      notify_result();
    }
  }

  void proactive_result_function() override
  {
    send_feedback();
    calculate_result();
    notify_check_finish();
  }

  void check_cancel_and_finish_proactive() override
  {
    RCLCPP_INFO(node_->get_logger(), "Check cancel and finish proactive function called");
    bool should_finish = yolo_state_->isCompleted();
    bool should_cancel = this->goal_handle_->is_canceling() || !this->goal_handle_->is_executing();

    if ((should_finish || should_cancel) && this->is_running()) {
      this->result_->action_server_cancel = this->server_goal_cancel_time_;
      this->result_->action_server_send_result = this->node_->now();
      if (should_cancel) {
        this->goal_handle_->canceled(this->result_);
      } else {
        this->goal_handle_->succeed(this->result_);
      }
      this->deactivate();
      RCLCPP_INFO(
        node_->get_logger(), "Proactive function finished, should finish: %d, should cancel: %d",
        should_finish, should_cancel);
    } else if (!this->is_running()) {
      RCLCPP_INFO(node_->get_logger(), "Proactive function finished previously");
    } else {
      notify_iteration();
    }
  }

  // ---------------- CUDA Function -----------------

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
      // sync does not call this function
    }

    if (this_ptr->processed_layers_ % this_ptr->batch_size_ == 0) {
      RCLCPP_INFO(this_ptr->node_->get_logger(), "Calculating result from callback function");
      if constexpr (isSyncAsync) {
        this_ptr->notify_result();
      }
    }
  }

  // ----------------- Common Functions -----------------

  void start() override
  {
    RCLCPP_INFO(node_->get_logger(), "Start function called");
    notify_iteration();
  }

  void compute() override
  {
    RCLCPP_INFO(node_->get_logger(), "Compute function called");

    // Start timing
    auto start_time = this->node_->now();

    // Determine synchronicity before the loop
    constexpr bool is_sync_async = isSyncAsync;

    for (int i = 0; i < batch_size_; i++) {
      RCLCPP_INFO(node_->get_logger(), "Computing batch part %d", i);

      void (*callback)(void *);
      if constexpr (!isSyncAsync) {
        callback = nullptr;
      } else if constexpr (isSyncAsync) {
        callback = forward_finished_callback;
      }
      RCLCPP_INFO(node_->get_logger(), "Callback function is null: %d", callback == nullptr);
      RCLCPP_INFO(node_->get_logger(), "Calling inferStep");

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

  void send_feedback() override
  {
    RCLCPP_INFO(node_->get_logger(), "Send feedback function called");
    auto feedback = std::make_shared<Anytime::Feedback>();
    // --- CUSTOM ---
    feedback->processed_layers = processed_layers_;
    // --- CUSTOM ---
    RCLCPP_INFO(node_->get_logger(), "Sending feedback, processed layers: %d", processed_layers_);
    this->goal_handle_->publish_feedback(feedback);
    RCLCPP_INFO(
      node_->get_logger(), "Proactive function feedback sent, processed layers: %d",
      processed_layers_);
  }

  void calculate_result() override
  {
    RCLCPP_INFO(node_->get_logger(), "Calculate result function called");
    auto new_result = std::make_shared<Anytime::Result>();

    RCLCPP_INFO(node_->get_logger(), "Calculating result");
    result_processed_layers_ = processed_layers_;
    std::vector<float> yolo_result;
    if constexpr (isReactiveProactive) {
      RCLCPP_INFO(node_->get_logger(), "Calculating latest exit");
      yolo_result = yolo_.calculateLatestExit(*yolo_state_);
    } else if constexpr (!isReactiveProactive) {
      RCLCPP_INFO(node_->get_logger(), "Finishing early");
      yolo_result = yolo_.finishEarly(*yolo_state_);
    }

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
      new_result->detections.push_back(detection);
    }

    // Add additional information to result
    new_result->average_batch_time = average_computation_time_;
    new_result->batch_size = batch_size_;
    new_result->processed_layers = processed_layers_;
    new_result->result_processed_layers = result_processed_layers_;

    new_result->action_server_receive = this->server_goal_receive_time_;
    new_result->action_server_accept = this->server_goal_accept_time_;
    new_result->action_server_start = this->server_goal_start_time_;

    this->result_ = new_result;
  }

  // Cancel function
  void notify_cancel() override
  {
    this->server_goal_cancel_time_ = this->node_->now();
    RCLCPP_INFO(node_->get_logger(), "Notify cancel function");
    notify_check_finish();
    RCLCPP_INFO(node_->get_logger(), "Notify cancel function finished");
  }

  // Reset function
  void reset() override
  {
    RCLCPP_INFO(node_->get_logger(), "Reset function called");
    this->result_ = std::make_shared<Anytime::Result>();

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

      cv::Mat blob = cv::dnn::blobFromImage(
        img,                  // input image
        1.0 / 255.0,          // scale factor (normalization)
        cv::Size(640, 640),   // output size
        cv::Scalar(0, 0, 0),  // mean subtraction (none here)
        true,                 // swapRB - converts BGR to RGB
        false                 // crop - no cropping
      );

      if (halfPrecision) {
        blob.convertTo(blob, CV_16F);  // Convert to half precision
      } else {
        blob.convertTo(blob, CV_32F);  // Convert to float
      }

      const size_t data_size =
        blob.total() * blob.elemSize();  // Total number of elements * size of each element

      // check if the buffer is large enough
      if (data_size > input_cuda_buffer_.size) {
        std::cerr << "Buffer size is not large enough" << std::endl;
        throw std::runtime_error("Buffer size is not large enough");
      }
      // Copy data to the buffer
      if (!input_cuda_buffer_.copyFromHost(blob.data, data_size)) {
        std::cerr << "Error copying data to buffer" << std::endl;
        throw std::runtime_error("Error copying data to buffer");
      }
    }

    // Reset YOLO state
    yolo_state_->restart(input_cuda_buffer_);

    this->result_->action_server_receive = this->server_goal_receive_time_;
    this->result_->action_server_accept = this->server_goal_accept_time_;
    this->result_->action_server_start = this->server_goal_start_time_;

    server_goal_cancel_time_ = this->node_->now();

    batch_count_ = 0;
    processed_layers_ = 0;  // Reset processed layers counter
    result_processed_layers_ = 0;
    average_computation_time_ = rclcpp::Duration(0, 0);
  }

protected:
  rclcpp::Node * node_;       // Node reference for logging
  int batch_size_;            // Batch size for compute iterations
  std::string weights_path_;  // Path to YOLO weights

  AnytimeYOLO yolo_;
  std::unique_ptr<InferenceState> yolo_state_;  // YOLO inference state as pointer
  CudaHostBuffer input_cuda_buffer_;            // Input image buffer

  bool halfPrecision = false;  // Flag for half precision

  // Batch count and average computation time
  int batch_count_ = 0;
  int processed_layers_ = 0;         // Counter for processed network layers
  int result_processed_layers_ = 0;  // Counter for processed network layers in result
  rclcpp::Duration average_computation_time_{0, 0};  // in milliseconds
};

#endif  // ANYTIME_MANAGEMENT_HPP