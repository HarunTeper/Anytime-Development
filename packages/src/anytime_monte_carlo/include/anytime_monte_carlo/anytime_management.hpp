#ifndef ANYTIME_MANAGEMENT_HPP
#define ANYTIME_MANAGEMENT_HPP

#include "anytime_interfaces/action/monte_carlo.hpp"
#include "anytime_monte_carlo/anytime_base.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <cstdint>
#include <future>
#include <random>

// Aliases for better readability
using Anytime = anytime_interfaces::action::MonteCarlo;
using AnytimeGoalHandle = rclcpp_action::ServerGoalHandle<Anytime>;

// Anytime Management class template
template <bool isReactiveProactive>
class AnytimeManagement : public AnytimeBase<double, Anytime, AnytimeGoalHandle>
{
public:
  // Constructor
  AnytimeManagement(rclcpp::Node * node, int batch_size = 1) : node_(node), batch_size_(batch_size)
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
      anytime_check_finish_waitable_ =
        std::make_shared<AnytimeWaitable>([this]() { this->check_cancel_and_finish_reactive(); });

      node_->get_node_waitables_interface()->add_waitable(
        anytime_iteration_waitable_, compute_callback_group_);
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
    send_feedback();
    notify_check_finish();
  }

  void check_cancel_and_finish_reactive() override
  {
    RCLCPP_INFO(node_->get_logger(), "Check cancel and finish reactive function called");
    bool should_finish = loop_count_ >= this->goal_handle_->get_goal()->goal;
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
      RCLCPP_DEBUG(
        node_->get_logger(), "Reactive function finished, should finish: %d, should cancel: %d",
        should_finish, should_cancel);
    } else if (!this->is_running()) {
      RCLCPP_DEBUG(node_->get_logger(), "Reactive function finished previously");
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
    send_feedback();
    calculate_result();
    notify_check_finish();
  }

  void check_cancel_and_finish_proactive() override
  {
    RCLCPP_INFO(node_->get_logger(), "Check cancel and finish proactive function called");
    bool should_finish = loop_count_ >= this->goal_handle_->get_goal()->goal;
    bool should_cancel = this->goal_handle_->is_canceling();

    if ((should_finish || should_cancel) && this->is_running()) {
      this->result_->action_server_cancel = this->server_goal_cancel_time_;
      this->result_->action_server_send_result = this->node_->now();
      if (should_cancel) {
        this->goal_handle_->canceled(this->result_);
      } else {
        this->goal_handle_->succeed(this->result_);
      }
      this->deactivate();
      RCLCPP_DEBUG(
        node_->get_logger(), "Reactive function finished, should finish: %d, should cancel: %d",
        should_finish, should_cancel);
    } else if (!this->is_running()) {
      RCLCPP_DEBUG(node_->get_logger(), "Reactive function finished previously");
    } else {
      notify_iteration();
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

    for (int i = 0; i < batch_size_; i++) {
      x = (float)rand() / RAND_MAX;
      y = (float)rand() / RAND_MAX;

      if (sqrt(pow(x, 2) + pow(y, 2)) <= 1) {
        count_inside_++;
      }
      count_total_++;
      loop_count_++;
    }

    // End timing
    auto end_time = this->node_->now();
    // Calculate computation time for this batch
    rclcpp::Duration computation_time = end_time - start_time;
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
    feedback->feedback = count_total_;
    // --- CUSTOM ---
    this->goal_handle_->publish_feedback(feedback);
    RCLCPP_DEBUG(
      node_->get_logger(), "Proactive function feedback sent, count total: %d", count_total_);
  }

  void calculate_result() override
  {
    RCLCPP_INFO(node_->get_logger(), "Calculate result function called");
    auto new_result = std::make_shared<Anytime::Result>();

    // Calculate the result
    new_result->result = 4 * (double)count_inside_ / count_total_;
    new_result->iterations = loop_count_;

    // Add additional information to result
    new_result->batch_time = average_computation_time_;
    new_result->batch_size = batch_size_;

    new_result->action_server_receive = this->server_goal_receive_time_;
    new_result->action_server_accept = this->server_goal_accept_time_;
    new_result->action_server_start = this->server_goal_start_time_;
    new_result->action_server_cancel = this->result_->action_server_cancel;

    this->result_ = new_result;
  }

  // Cancel function
  void notify_cancel() override
  {
    server_goal_cancel_time_ = this->node_->now();
    RCLCPP_INFO(node_->get_logger(), "Notify cancel function");
    notify_check_finish();
    RCLCPP_INFO(node_->get_logger(), "Notify cancel function finished");
  }

  // Reset function
  void reset() override
  {
    RCLCPP_INFO(node_->get_logger(), "Reset function called");
    this->result_ = std::make_shared<Anytime::Result>();

    // Reset the count variables
    count_total_ = 0;
    count_inside_ = 0;
    count_outside_ = 0;

    loop_count_ = 0;

    this->result_->action_server_receive = this->server_goal_receive_time_;
    this->result_->action_server_accept = this->server_goal_accept_time_;
    this->result_->action_server_start = this->server_goal_start_time_;

    this->server_goal_cancel_time_ = this->node_->now();

    batch_count_ = 0;
    average_computation_time_ = rclcpp::Duration(0, 0);
  }

protected:
  rclcpp::Node * node_;  // Node reference for logging
  int batch_size_;       // Batch size for compute iterations

  // Count variables
  int count_total_ = 0;
  int count_inside_ = 0;
  int count_outside_ = 0;

  int loop_count_ = 0;

  float x = 0.0;
  float y = 0.0;

  // Batch count and average computation time
  int batch_count_ = 0;
  rclcpp::Duration average_computation_time_{0, 0};  // in milliseconds
};

#endif  // MONTE_CARLO_HPP