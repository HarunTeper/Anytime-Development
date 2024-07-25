#include "yolo_action/anytime_server.hpp"

#include <c10/cuda/CUDAStream.h>

AnytimeActionServer::AnytimeActionServer(const rclcpp::NodeOptions& options)
    : Node("anytime_action_server", options), model(create_config()) {
  RCLCPP_INFO(this->get_logger(), "Starting Anytime action server");
  action_server_ =
      rclcpp_action::create_server<anytime_interfaces::action::Anytime>(
          this, "anytime",
          [this](
              const rclcpp_action::GoalUUID uuid,
              std::shared_ptr<const anytime_interfaces::action::Anytime::Goal>
                  goal) { return this->handle_goal(uuid, goal); },
          [this](const std::shared_ptr<AnytimeGoalHandle> goal_handle) {
            return this->handle_cancel(goal_handle);
          },
          [this](const std::shared_ptr<AnytimeGoalHandle> goal_handle) {
            return this->handle_accepted(goal_handle);
          });
  RCLCPP_INFO(this->get_logger(), "Anytime action server started");

  callback_group1_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(0), [this]() { execute(); }, callback_group1_);
  timer_->cancel();

  start_time = this->now();
  end_time = this->now();
}

AnytimeActionServer::~AnytimeActionServer() {}

YOLOModelConfig AnytimeActionServer::create_config() {
  this->declare_parameter<std::string>("weights_path");

  auto weights_path = this->get_parameter("weights_path").as_string();
  RCLCPP_INFO(this->get_logger(), "Weights path1: %s", weights_path.c_str());
  YOLOModelConfig config{
      .weights_path = weights_path + "/yolov7-tiny-traced-half.pt",
      .source_info_path = weights_path + "/yolov7-tiny-traced.json",
      .stream = c10::cuda::getDefaultCUDAStream()};
  RCLCPP_INFO(this->get_logger(), "Weights path2: %s",
              config.weights_path.c_str());
  return config;
}

rclcpp_action::GoalResponse AnytimeActionServer::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    const std::shared_ptr<const anytime_interfaces::action::Anytime::Goal>
        goal) {
  RCLCPP_INFO(this->get_logger(), "Received goal request with number %d",
              goal->goal);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AnytimeActionServer::handle_cancel(
    const std::shared_ptr<AnytimeGoalHandle> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void AnytimeActionServer::execute(
    const std::shared_ptr<AnytimeGoalHandle> goal_handle,
    std::shared_ptr<rclcpp::TimerBase> timer) {
  (void)timer;
  if (this->forward_one()) {
    end_time = this->now();
    // //print time difference
    RCLCPP_INFO(this->get_logger(), "Time difference: %f",
                (end_time - start_time).seconds());
    auto test_tensor = torch::randn({1, 3, 640, 640}).cuda().to(torch::kHalf);
    this->initialize_forward(test_tensor);
    start_time = this->now();
  }

  // RCLCPP_INFO(this->get_logger(), "Executing goal");
  // auto feedback =
  // std::make_shared<anytime_interfaces::action::Anytime::Feedback>(); auto &
  // feedback_value = feedback->feedback; feedback_value = 0; auto result =
  // std::make_shared<anytime_interfaces::action::Anytime::Result>();
  // result->result = 0;

  // int count_total = 0;
  // int count_inside = 0;
  // int count_outside = 0;

  // float_t x = 0.0;
  // float_t y = 0.0;

  // for (int i = 1; i <= goal_handle->get_goal()->goal; i++){
  //     if (goal_handle->is_canceling()){
  //         RCLCPP_INFO(this->get_logger(), "Goal was canceled");
  //         result->result = feedback_value;
  //         goal_handle->canceled(result);
  //         return;
  //     }
  //      // sample x and y between 0 and 1, and if the length of the vector is
  //      greater than one, add count to count_outside, otherwise add to
  //      count_inside
  //     x = (float_t)rand() / RAND_MAX;
  //     y = (float_t)rand() / RAND_MAX;

  //     if (sqrt(pow(x, 2) + pow(y, 2)) <= 1){
  //         count_inside++;
  //     } else {
  //         count_outside++;
  //     }
  //     count_total++;

  //     feedback_value = 4 * (float_t)count_inside / count_total;

  //     // goal_handle->publish_feedback(feedback);

  // }

  // result->result = feedback_value;

  // goal_handle->succeed(result);

  // RCLCPP_INFO(this->get_logger(), "Goal was completed");

  // // timer->cancel();
}

void AnytimeActionServer::execute() {
  if (this->forward_one()) {
    end_time = this->now();
    // //print time difference
    RCLCPP_INFO(this->get_logger(), "Time difference: %f",
                (end_time - start_time).seconds());
    auto result =
        std::make_shared<anytime_interfaces::action::Anytime::Result>();
    result->result = 1;
    goal_handle_->succeed(result);
    timer_->cancel();
  }
}

bool AnytimeActionServer::forward_one() {
  // RCLCPP_INFO(this->get_logger(), "Forward one");
  return model.forward_one();
}

void AnytimeActionServer::initialize_forward(torch::Tensor x) {
  model.forward_init(x);
}

void AnytimeActionServer::handle_accepted(
    const std::shared_ptr<AnytimeGoalHandle> goal_handle) {
  RCLCPP_INFO(this->get_logger(),
              "Goal accepted by server, waiting for result");
  // std::thread{std::bind(&AnytimeActionServer::execute, this, goal_handle,
  // nullptr)}.detach();

  // create a timer and insert it into the vector
  // std::shared_ptr<rclcpp::TimerBase> timer;
  // timer = this->create_wall_timer(std::chrono::seconds(1), [this, timer,
  // goal_handle](){
  //     this->execute(goal_handle, timer);
  // });
  // timers_.push_back(timer);

  goal_handle_ = goal_handle;

  if (timer_->is_canceled()) {
    tensor_ = torch::randn({1, 3, 640, 640}).cuda().to(torch::kHalf);
    RCLCPP_INFO(this->get_logger(), "Tensor initialized");
    initialize_forward(tensor_);
    RCLCPP_INFO(this->get_logger(), "Forward initialized");

    timer_->reset();
    start_time = this->now();
    RCLCPP_INFO(this->get_logger(), "Timer reset");
  }
}