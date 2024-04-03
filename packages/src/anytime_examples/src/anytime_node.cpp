#include "anytime_examples/anytime_node.hpp"

AnytimeST::AnytimeST() : Node("anytime")
{
    std::cout << "Single threaded Node Started" << std::endl;

    // Start the resource thread
    resource_thread_ = std::thread(&AnytimeST::resource_thread, this);
}

AnytimeST::~AnytimeST()
{
    // Join the resource thread
    resource_thread_.join();
}

void AnytimeST::resource_thread()
{
    std::cout << "Resource Thread Started" << std::endl;
    rclcpp::Rate rate(1);
    while (rclcpp::ok())
    {
        std::cout << "Resource Thread Running" << std::endl;
        rate.sleep();
    }
}

AnytimeMT::AnytimeMT() : Node("anytime")
{
    std::cout << "Multi threaded Node Started" << std::endl;
}