#include "yolo_rt/yolo_node.hpp"

int main(int argc, char * argv[])
{
    //print argv
    for (int i = 0; i < argc; i++)
    {
        std::cout << "argv[" << i << "] = " << argv[i] << std::endl;
    }
    
    rclcpp::init(argc, argv);

    if (argc > 1 && strcmp(argv[1], "single") == 0)
    {
        std::cout << "Single threaded" << std::endl;
        auto node = std::make_shared<YOLONodeST>();
        rclcpp::spin(node);
    }
    else
    {
        std::cout << "Multi threaded" << std::endl;
        auto node = std::make_shared<YOLONodeMT>();
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
        executor.add_node(node);
        executor.spin();
    }

    rclcpp::shutdown();
    return 0;
}