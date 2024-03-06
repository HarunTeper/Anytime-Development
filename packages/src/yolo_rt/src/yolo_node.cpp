#include "yolo_rt/yolo_node.hpp"
#include <ATen/core/TensorBody.h>
#include <ATen/core/ivalue.h>
#include <torch/script.h>

//include for CUDAStream
#include <c10/cuda/CUDAStream.h>

YOLONodeST::YOLONodeST() : Node("yolo"),
    model(create_config()) 
{
    RCLCPP_INFO(this->get_logger(), "Initializing YOLONode");
    auto test_tensor = torch::randn({1, 3, 640, 640}).cuda().to(torch::kHalf);
    RCLCPP_INFO(this->get_logger(), "Initializing forward");
    this->initialize_forward(test_tensor);
    RCLCPP_INFO(this->get_logger(), "Forward initialized");
    //initialize timer
    timer_ = this->create_wall_timer(std::chrono::milliseconds(0),std::bind(&YOLONodeST::timer_forward_callback, this));
    //set start time
    start_time = this->now();
    // for (int i = 0; i < 250; i++){
    //     auto test_tensor = torch::randn({1, 3, 640, 640}).cuda().to(torch::kHalf);
    //     start_time = this->now();
    //     this->model.forward_full_at_once({test_tensor});
    //     end_time = this->now();
    //     RCLCPP_INFO(this->get_logger(), "Time difference full at once: %f", (end_time - start_time).seconds());
    // }
    // for (int i = 0; i < 500; i++){
    //     auto test_tensor = torch::randn({1, 3, 640, 640}).cuda().to(torch::kHalf);
    //     start_time = this->now();
    //     this->model.forward_full({test_tensor});
    //     end_time = this->now();
    //     RCLCPP_INFO(this->get_logger(), "Time difference full: %f", (end_time - start_time).seconds());
    // }

}

YOLOModelConfig YOLONodeST::create_config(){
    this->declare_parameter<std::string>("weights_path");

    auto weights_path   = this->get_parameter("weights_path").as_string();
    RCLCPP_INFO(this->get_logger(), "Weights path1: %s", weights_path.c_str());
    YOLOModelConfig config { 
        .weights_path =  weights_path + "/yolov7-tiny-traced-half.pt", 
        .source_info_path = weights_path + "/yolov7-tiny-traced.json", 
        .stream = c10::cuda::getDefaultCUDAStream() 
    };
    RCLCPP_INFO(this->get_logger(), "Weights path2: %s", config.weights_path.c_str());
    return config;
}

void YOLONodeST::timer_forward_callback()
{
    if (this->forward_one()){
        end_time = this->now();
        // //print time difference
        RCLCPP_INFO(this->get_logger(), "Time difference: %f", (end_time - start_time).seconds());
        auto test_tensor = torch::randn({1, 3, 640, 640}).cuda().to(torch::kHalf);
        this->initialize_forward(test_tensor);
        start_time = this->now();
    }
}

torch::IValue YOLONodeST::forward(torch::Tensor x)
{
    return model.forward_full(x);
}

void YOLONodeST::initialize_forward(torch::Tensor x)
{
    model.forward_init(x);
}

bool YOLONodeST::forward_one(){
    // RCLCPP_INFO(this->get_logger(), "Forward one");
    return model.forward_one();
}

//subscription callback camera_callback
void YOLONodeST::camera_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received image");
    // //use torch from_blob to convert image to tensor
    // auto tensor = torch::from_blob((void*)msg->data.data(), {msg->height, msg->width, 3}, torch::kByte);
    // //forward
    // this->initialize_forward(tensor);
}

YOLONodeMT::YOLONodeMT() : Node("yolo"),
    model(create_config()) 
{
    RCLCPP_INFO(this->get_logger(), "Initializing YOLONode");
    auto test_tensor = torch::randn({1, 3, 640, 640}).cuda().to(torch::kHalf);
    RCLCPP_INFO(this->get_logger(), "Initializing forward");
    this->initialize_forward(test_tensor);
    RCLCPP_INFO(this->get_logger(), "Forward initialized");
    //create two mutually exclusive callback groups
    callback_group1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    //initialize timer with callback group
    timer_ = this->create_wall_timer(std::chrono::milliseconds(0),std::bind(&YOLONodeMT::timer_forward_callback, this), callback_group1_);
    //create subscription options with callback group 2
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group2_;
    //initialize subscription with callback group 2
    // sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw", 10, std::bind(&YOLONodeMT::camera_callback, this, std::placeholders::_1), options);
    fake_sub = this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&YOLONodeMT::camera_callback, this), callback_group2_);

    //set start time
    start_time = this->now();
}

YOLOModelConfig YOLONodeMT::create_config(){
    this->declare_parameter<std::string>("weights_path");

    auto weights_path   = this->get_parameter("weights_path").as_string();
    RCLCPP_INFO(this->get_logger(), "Weights path1: %s", weights_path.c_str());
    YOLOModelConfig config { 
        .weights_path =  weights_path + "/yolov7-tiny-traced-half.pt", 
        .source_info_path = weights_path + "/yolov7-tiny-traced.json", 
        .stream = c10::cuda::getDefaultCUDAStream() 
    };
    RCLCPP_INFO(this->get_logger(), "Weights path2: %s", config.weights_path.c_str());
    return config;
}

void YOLONodeMT::initialize_forward(torch::Tensor x)
{
    model.forward_init(x);
}

void YOLONodeMT::timer_forward_callback()
{
    if (!this->forwarding && this->start_queued) {
        RCLCPP_INFO(this->get_logger(), "Forwarding");
        this->forwarding = true;
        {
            auto mtx_guard = std::lock_guard<std::mutex>(this->mtx);
            this->start_queued = false;
            this->cancel_queued = false;

            auto x = this->queued_tensor;
            this->model.forward_init(x);
        }
        start_time = this->now();
        // with torch.no_grad
        torch::NoGradGuard no_grad;

        while (!this->cancel_queued && !this->model.forward_one_blocking()) {
        }

        end_time = this->now();
        // //print time difference
        RCLCPP_INFO(this->get_logger(), "Time difference: %f", (end_time - start_time).seconds());

        RCLCPP_INFO(this->get_logger(), "Cancelled: %d", this->cancel_queued);
        RCLCPP_INFO(this->get_logger(), "Layer %d", (int) this->model.outputs.size());

        this->forwarding = false;
    }
}

bool YOLONodeMT::forward_one(){
    // RCLCPP_INFO(this->get_logger(), "Forward one");
    return model.forward_one();
}

void YOLONodeMT::camera_callback()
{
    RCLCPP_INFO(this->get_logger(), "Received image");
    //use torch from_blob to convert image to tensor
    // auto tensor = torch::from_blob((void*)msg->data.data(), {msg->height, msg->width, 3}, torch::kByte);
    auto tensor = torch::randn({1, 3, 640, 640}).cuda().to(torch::kHalf);
    
    {
        auto mtx_guard = std::lock_guard<std::mutex>(this->mtx);
        this->start_queued = true;
        this->cancel_queued = true;
        this->queued_tensor = tensor;
    }
}

//subscription callback camera_callback
// void YOLONodeMT::camera_callback(const sensor_msgs::msg::Image::SharedPtr msg)
// {
//     RCLCPP_INFO(this->get_logger(), "Received image");
//     //use torch from_blob to convert image to tensor
//     // auto tensor = torch::from_blob((void*)msg->data.data(), {msg->height, msg->width, 3}, torch::kByte);
//     auto tensor = torch::randn({1, 3, 640, 640}).cuda().to(torch::kHalf);
//     //forward
//     this->initialize_forward(tensor);
// }