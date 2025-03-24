#ifndef ANYTIME_MANAGEMENT_HPP
#define ANYTIME_MANAGEMENT_HPP

#include "anytime_yolo/anytime_base.hpp"
#include "anytime_yolo/yolo.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>

#include <cstdint>
#include <future>
#include <random>

// Aliases for better readability
using Anytime = anytime_interfaces::action::Yolo;
using AnytimeGoalHandle = rclcpp_action::ServerGoalHandle<Anytime>;

// Anytime Management class template
template <bool isReactiveProactive, bool isSingleMulti>
class AnytimeManagement : public AnytimeBase<double, Anytime, AnytimeGoalHandle>
{
public:
  // Constructor
  AnytimeManagement(rclcpp::Node * node, int batch_size = 1, const std::string & weights_path = "")
  : node_(node), batch_size_(batch_size), weights_path_(weights_path)
  {
    // print path
    RCLCPP_INFO(node_->get_logger(), "Weights path: %s", weights_path_.c_str());
    // Initialize yolo
    AnytimeYOLO yolo(weights_path_, false);

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
    if (check_cancel_and_finish_reactive()) {
      return;
    } else {
      compute();
      notify_iteration();
    }
  }

  void reactive_function_loop()
  {
    while (true) {
      if (check_cancel_and_finish_reactive()) {
        return;
      } else {
        compute();
      }
    }
  }

  bool check_cancel_and_finish_reactive() override
  {
    // bool should_finish = loop_count_ >= this->goal_handle_->get_goal()->goal;
    bool should_cancel = this->goal_handle_->is_canceling();

    // if (should_finish || should_cancel) {
    if (should_cancel) {
      this->result_->action_server_send_result = this->node_->now();
      this->result_->batch_time = this->average_computation_time_;
      this->calculate_result();

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
  void proactive_function()
  {
    compute();
    calculate_result();
    notify_check_finish();
  }

  void check_cancel_and_finish_proactive() override
  {
    // bool should_finish = loop_count_ >= this->goal_handle_->get_goal()->goal;
    bool should_cancel = this->goal_handle_->is_canceling() || !this->goal_handle_->is_executing();

    // if ((should_finish || should_cancel) && this->is_running()) {
    if ((should_cancel) && this->is_running()) {
      this->result_->action_server_send_result = this->node_->now();

      this->result_->batch_time = average_computation_time_;

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

  void compute() override
  {
    // Start timing
    auto start_time = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < batch_size_; i++) {
      yolo.inferStep(state);
    }

    // End timing
    auto end_time = std::chrono::high_resolution_clock::now();
    // Calculate computation time for this batch
    auto duration = end_time - start_time;
    auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
    rclcpp::Time computation_time = rclcpp::Time(duration_ns);

    // Update the average computation time
    batch_count_++;
    if (batch_count_ == 1) {
      average_computation_time_ = computation_time;
    } else {
      average_computation_time_ = rclcpp::Time(
        average_computation_time_.nanoseconds() +
        (computation_time.nanoseconds() - average_computation_time_.nanoseconds()) / batch_count_);
    }
  }

  void calculate_result() override
  {
    std::vector<std::vector<float>> results;
    results.push_back(yolo.finishEarly(state));

    // Add additional information to result
    this->result_->batch_size = batch_size_;
    this->result_->is_reactive_proactive = isReactiveProactive;
    this->result_->is_single_multi = isSingleMulti;
  }

  // Cancel function
  void notify_cancel() override
  {
    this->result_->action_server_cancel = this->node_->now();
    if constexpr (isReactiveProactive) {
      notify_check_finish();
    } else if constexpr (!isReactiveProactive) {
      // nothing to do
    }
  }

  // Reset function
  void reset() override
  {
    input_image_.free();

    // Process image data from the goal handle
    if (this->goal_handle_) {
      const auto & goal = this->goal_handle_->get_goal();

      // Make sure we have image data
      if (goal->data.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "No image data in goal");
        return;
      }

      // Create OpenCV matrix from raw data
      cv::Mat img(
        goal->height, goal->width, CV_8UC(goal->channels),
        const_cast<uint8_t *>(goal->data.data()));

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
      if (!input_image_.allocate(data_size)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to allocate CUDA buffer");
        return;
      }

      if (!input_image_.copyFromHost(nchw_data.data(), data_size)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to copy data to CUDA buffer");
        return;
      }
    }

    // Reset YOLO state
    yolo_state_ = yolo.createInferenceState(input);

    this->result_->action_server_receive = this->server_goal_receive_time_;
    this->result_->action_server_accept = this->server_goal_accept_time_;
    this->result_->action_server_start = this->server_goal_start_time_;

    batch_count_ = 0;
    average_computation_time_ = rclcpp::Time(0, 0);
  }

protected:
  rclcpp::Node * node_;       // Node reference for logging
  int batch_size_;            // Batch size for compute iterations
  std::string weights_path_;  // Path to YOLO weights

  CudaBuffer input_image_;
  InferenceState yolo_state_;

  // Batch count and average computation time
  int batch_count_ = 0;
  rclcpp::Time average_computation_time_;  // in milliseconds
};

#endif  // ANYTIME_MANAGEMENT_HPP