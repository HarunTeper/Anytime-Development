#ifndef ANYTIME_BASE_HPP
#define ANYTIME_BASE_HPP

#include "anytime_monte_carlo/anytime_waitable.hpp"

template <typename ReturnType, typename InterfaceType, typename GoalHandleType>
class AnytimeBase {
 public:
  virtual ~AnytimeBase() = default;

  virtual void reset() = 0;
  virtual bool check_cancel() = 0;
  virtual void finish() = 0;

  void activate() { is_running_ = true; }
  void deactivate() { is_running_ = false; }
  bool is_active() { return is_running_; }

  void set_goal_handle(std::shared_ptr<GoalHandleType> goal_handle) {
    goal_handle_ = goal_handle;
  }

  void set_anytime_waitable(std::shared_ptr<AnytimeWaitable> anytime_waitable) {
    anytime_waitable_ = anytime_waitable;
  }

  std::shared_ptr<AnytimeWaitable> get_anytime_waitable() {
    return anytime_waitable_;
  }

  std::shared_ptr<AnytimeWaitable> get_anytime_finish_waitable() {
    return anytime_finish_waitable_;
  }

  void notify() { anytime_waitable_->notify(); }

  void notify_finish() { anytime_finish_waitable_->notify(); }

  void cancel() { cancel_ = true; }

  void unset_cancel() { cancel_ = false; }

 protected:
  std::shared_ptr<AnytimeWaitable> anytime_waitable_;
  std::shared_ptr<AnytimeWaitable> anytime_finish_waitable_ =
      std::make_shared<AnytimeWaitable>([this]() { this->finish(); });
  bool is_running_ = false;
  bool cancel_ = false;

  // goal handle
  std::shared_ptr<GoalHandleType> goal_handle_;

  std::shared_ptr<typename InterfaceType::Feedback> feedback_ =
      std::make_shared<typename InterfaceType::Feedback>();

  std::shared_ptr<typename InterfaceType::Result> result_ =
      std::make_shared<typename InterfaceType::Result>();
};

#endif  // ANYTIME_BASE_HPP
