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
template <bool isActive, bool separate_thread, bool multi_threading>
class MonteCarloPi : public AnytimeBase<double, Anytime, AnytimeGoalHandle> {
 public:
  // Constructor
  MonteCarloPi(rclcpp::Node* node, int batch_size = 1)
      : node_(node), batch_size_(batch_size) {
    anytime_waitable_ = std::make_shared<AnytimeWaitable>([this]() {
      if constexpr (isActive) {
        if constexpr (multi_threading) {
          this->active_function_loop(this->goal_handle_);
        } else if constexpr (!multi_threading) {
          this->active_function(this->goal_handle_);
        }
      } else if constexpr (!isActive) {
        if constexpr (multi_threading) {
          this->passive_function_loop(this->goal_handle_);
        } else if constexpr (!multi_threading) {
          this->passive_function(this->goal_handle_);
        }
      }
    });

    anytime_finish_waitable_ = std::make_shared<AnytimeWaitable>([this]() {
      if constexpr (isActive) {
        this->check_cancel_and_finish_active();
      } else if constexpr (!isActive) {
        this->check_cancel_and_finish_passive();
      }
    });

    // callback group
    compute_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // add the waitable to the node
    node_->get_node_waitables_interface()->add_waitable(
        anytime_waitable_, compute_callback_group_);

    node_->get_node_waitables_interface()->add_waitable(
        anytime_finish_waitable_,
        node_->get_node_base_interface()->get_default_callback_group());
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
  void active_function(const std::shared_ptr<AnytimeGoalHandle> goal_handle) {
    if (check_cancel_and_finish_active()) {
      return;
    }
    if (loop_count_ < goal_handle->get_goal()->goal) {
      compute();

      this->anytime_waitable_->notify();

    } else {
      active_finish();
    }
  }

  // Blocking function to approximate Pi
  void active_function_loop(
      const std::shared_ptr<AnytimeGoalHandle> goal_handle) {
    while (true) {
      if (check_cancel_and_finish_active()) {
        break;
      }
      if (loop_count_ < goal_handle->get_goal()->goal) {
        compute();
      } else {
        active_finish();
        break;
      }
    }
  }

  void active_finish() {
    RCLCPP_INFO(node_->get_logger(), "Goal was completed");
    finished_ = true;
    check_cancel_and_finish_active();
  }

  // Passive function to approximate Pi
  void passive_function(const std::shared_ptr<AnytimeGoalHandle> goal_handle) {
    if (loop_count_ < goal_handle->get_goal()->goal && !canceled_) {
      compute();

      calculate_result();

      this->anytime_waitable_->notify();

    } else {
      passive_function_finish();
    }
  }

  void passive_function_loop(
      const std::shared_ptr<AnytimeGoalHandle> goal_handle) {
    while (true) {
      if (loop_count_ < goal_handle->get_goal()->goal && !canceled_) {
        compute();

        calculate_result();
      } else {
        passive_function_finish();
        break;
      }
    }
  }

  void passive_function_finish() {
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

  void calculate_result() override {
    // Calculate the result
    auto start_time = std::chrono::high_resolution_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::high_resolution_clock::now() - start_time)
               .count() < 10) {
      // Spin for 5 ms
    }

    this->result_->action_compute_end = this->node_->now();
    this->result_->result = 4 * (double)count_inside_ / count_total_;
    this->result_->iterations = loop_count_;
  }

  bool check_cancel_and_finish_active() {
    if (this->goal_handle_->is_canceling()) {
      RCLCPP_INFO(node_->get_logger(), "Canceling goal");
      calculate_result();
      this->result_->action_end = this->node_->now();
      this->goal_handle_->canceled(this->result_);
      this->deactivate();
      return true;
    }
    if (finished_) {
      RCLCPP_INFO(node_->get_logger(), "Finishing Goal");
      calculate_result();
      this->result_->action_end = this->node_->now();
      this->goal_handle_->succeed(this->result_);
      this->deactivate();
      return true;
    }
    return false;
  }

  void check_cancel_and_finish_passive() {
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

  // Cancel function
  void cancel() override {
    this->result_->action_cancel = this->node_->now();
    if constexpr (isActive) {
      // nothing to do
    } else if constexpr (!isActive) {
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
        if constexpr (isActive) {
          this->active_function_loop(this->goal_handle_);
        } else if constexpr (!isActive) {
          this->passive_function_loop(this->goal_handle_);
        }
      }).detach();
    } else {
      this->notify();
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