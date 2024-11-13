#ifndef MONTE_CARLO_HPP
#define MONTE_CARLO_HPP

#include <cstdint>
#include <future>
#include <random>
#include "anytime_monte_carlo/anytime_base.hpp"

// Aliases for better readability
using Anytime = anytime_interfaces::action::Anytime;
using AnytimeGoalHandle = rclcpp_action::ServerGoalHandle<Anytime>;

// Monte Carlo Pi class template
template <bool isReactive, bool separate_thread, bool multi_threading>
class MonteCarloPi : public AnytimeBase<double, Anytime, AnytimeGoalHandle> {
 public:
  // Constructor
  MonteCarloPi(rclcpp::Node* node, int batch_size = 1)
      : node_(node), batch_size_(batch_size) {
    if constexpr (isReactive && multi_threading) {
      anytime_iteration_waitable_ = std::make_shared<AnytimeWaitable>(
          [this]() { this->reactive_function_loop(); });
    } else if constexpr (isReactive && !multi_threading) {
      anytime_iteration_waitable_ = std::make_shared<AnytimeWaitable>(
          [this]() { this->reactive_function(); });
    } else if constexpr (!isReactive && multi_threading) {
      anytime_iteration_waitable_ = std::make_shared<AnytimeWaitable>(
          [this]() { this->proactive_function_loop(this->goal_handle_); });
    } else if constexpr (!isReactive && !multi_threading) {
      anytime_iteration_waitable_ = std::make_shared<AnytimeWaitable>(
          [this]() { this->proactive_function(this->goal_handle_); });
    }

    if constexpr (isReactive) {
      anytime_result_waitable_ = std::make_shared<AnytimeWaitable>(
          [this]() { this->calculate_result_reactive(); });
    } else if constexpr (!isReactive) {
      anytime_result_waitable_ = std::make_shared<AnytimeWaitable>(
          [this]() { this->calculate_result(); });
    }

    anytime_result_waitable_ = std::make_shared<AnytimeWaitable>([this]() {
      if constexpr (isReactive) {
        this->calculate_result_reactive();
      } else if constexpr (!isReactive) {
        this->calculate_result();
      }
    });

    if constexpr (isReactive) {
      // nothing to do
    } else if constexpr (!isReactive) {
      anytime_finish_waitable_ = std::make_shared<AnytimeWaitable>(
          [this]() { this->check_cancel_and_finish_proactive(); });
    }

    // callback group
    compute_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    if constexpr (!separate_thread) {
      if constexpr (isReactive) {
        // add the waitable to the node
        node_->get_node_waitables_interface()->add_waitable(
            anytime_iteration_waitable_,
            node_->get_node_base_interface()->get_default_callback_group());

        node_->get_node_waitables_interface()->add_waitable(
            anytime_result_waitable_,
            node_->get_node_base_interface()->get_default_callback_group());
      } else if constexpr (!isReactive) {
        // add the waitable to the node
        node_->get_node_waitables_interface()->add_waitable(
            anytime_iteration_waitable_,
            node_->get_node_base_interface()->get_default_callback_group());

        node_->get_node_waitables_interface()->add_waitable(
            anytime_result_waitable_,
            node_->get_node_base_interface()->get_default_callback_group());

        node_->get_node_waitables_interface()->add_waitable(
            anytime_finish_waitable_,
            node_->get_node_base_interface()->get_default_callback_group());
      }
    }
  }

  void compute() {
    for (int i = 0; i < batch_size_; ++i) {
      x = (float)rand() / RAND_MAX;
      y = (float)rand() / RAND_MAX;

      if (sqrt(pow(x, 2) + pow(y, 2)) <= 1) {
        count_inside_++;
      }
      count_total_++;

      loop_count_++;
    }
  }

  // Blocking function to approximate Pi
  void reactive_function() {
    if (check_cancel_reactive()) {
      return;
    } else {
      compute();
      this->notify_result();
    }
  }

  void calculate_result_reactive() {
    if (check_cancel_reactive()) {
      return;
    } else {
      calculate_result();
      if (check_finish_reactive()) {
        return;
      } else {
        this->notify_iteration();
      }
    }
  }

  // Blocking function to approximate Pi
  void reactive_function_loop() {
    while (true) {
      if (check_cancel_reactive()) {
        return;
      } else {
        compute();
      }
      if (check_cancel_reactive()) {
        return;
      } else {
        calculate_result();
      }
      if (check_finish_reactive()) {
        return;
      }
    }
  }

  bool check_cancel_reactive() {
    if (this->goal_handle_->is_canceling()) {
      RCLCPP_INFO(node_->get_logger(), "Canceling goal");
      this->result_->action_end = this->node_->now();
      this->goal_handle_->canceled(this->result_);
      this->deactivate();
      return true;
    }
    return false;
  }

  bool check_finish_reactive() {
    if (loop_count_ >= this->goal_handle_->get_goal()->goal) {
      RCLCPP_INFO(node_->get_logger(), "Finishing Goal");
      this->result_->action_end = this->node_->now();
      this->goal_handle_->succeed(this->result_);
      this->deactivate();
      return true;
    }
    return false;
  }

  // proactive function to approximate Pi
  void proactive_function(
      const std::shared_ptr<AnytimeGoalHandle> goal_handle) {
    if (loop_count_ < goal_handle->get_goal()->goal && !canceled_) {
      compute();

      calculate_result();

      this->notify_iteration();

    } else {
      proactive_function_finish();
    }
  }

  void proactive_function_loop(
      const std::shared_ptr<AnytimeGoalHandle> goal_handle) {
    while (true) {
      if (loop_count_ < goal_handle->get_goal()->goal && !canceled_) {
        compute();

        calculate_result();
      } else {
        proactive_function_finish();
        break;
      }
    }
  }

  void proactive_function_finish() {
    RCLCPP_INFO(node_->get_logger(), "Goal was completed");
    {
      std::lock_guard<std::mutex> lock(this->notify_mutex_);
      if (!canceled_ && !finished_) {
        RCLCPP_INFO(node_->get_logger(), "Finishing Goal");
        this->result_->action_end = this->node_->now();
        this->goal_handle_->succeed(this->result_);
        finished_ = true;
        this->deactivate();
      } else if (canceled_ && finished_) {
        this->deactivate();
      } else if (canceled_ && !finished_) {
        finished_ = true;
      } else if (canceled_ && finished_) {
        RCLCPP_INFO(node_->get_logger(), "Finishing Goal");
        this->result_->action_end = this->node_->now();
        this->goal_handle_->succeed(this->result_);
        this->deactivate();
      }
    }
  }

  void check_cancel_and_finish_proactive() {
    std::lock_guard<std::mutex> lock(this->notify_mutex_);
    if (finished_) {
      RCLCPP_INFO(node_->get_logger(), "Finishing Goal");
      this->result_->action_end = this->node_->now();
      this->goal_handle_->succeed(this->result_);
      this->deactivate();
      return;
    } else {
      RCLCPP_INFO(node_->get_logger(), "Canceling goal");
      this->result_->action_end = this->node_->now();
      this->goal_handle_->canceled(this->result_);
      finished_ = true;
    }
  }

  void calculate_result() override {
    // Calculate the result
    this->result_->action_compute_end = this->node_->now();
    this->result_->result = 4 * (double)count_inside_ / count_total_;
    this->result_->iterations = loop_count_;
  }

  // Cancel function
  void cancel() override {
    this->result_->action_cancel = this->node_->now();
    if constexpr (isReactive) {
      // nothing to do
    } else if constexpr (!isReactive) {
      {
        std::lock_guard<std::mutex> lock(this->notify_mutex_);
        if (!finished_) {
          this->notify_finish();
          canceled_ = true;
        }
      }
    }
  }

  // Reset function
  void reset() override {
    // Reset the count variables
    count_total_ = 0;
    count_inside_ = 0;
    count_outside_ = 0;

    loop_count_ = 0;

    finished_ = false;
    canceled_ = false;

    this->result_->client_start = this->goal_handle_->get_goal()->client_start;
    this->result_->action_send = this->goal_handle_->get_goal()->action_send;
    this->result_->action_accept = this->goal_handle_accept_time_;
  }

  void start() override {
    this->result_->action_start = node_->now();
    if constexpr (separate_thread) {
      std::thread([this]() {
        if constexpr (isReactive) {
          this->reactive_function_loop();
        } else if constexpr (!isReactive) {
          this->proactive_function_loop(this->goal_handle_);
        }
      }).detach();
    } else {
      this->notify_iteration();
    }
  }

 protected:
  rclcpp::Node* node_;  // Node reference for logging
  int batch_size_;      // Batch size for compute iterations

  // Count variables
  int count_total_ = 0;
  int count_inside_ = 0;
  int count_outside_ = 0;

  int loop_count_ = 0;

  float x = 0.0;
  float y = 0.0;
};

#endif  // MONTE_CARLO_HPP