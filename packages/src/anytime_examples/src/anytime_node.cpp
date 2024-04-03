#include "anytime_examples/anytime_node.hpp"

AnytimeST::AnytimeST() : Node("anytime")
{
    std::cout << "Single threaded Node Started" << std::endl;
}

AnytimeMT::AnytimeMT() : Node("anytime")
{
    std::cout << "Multi threaded Node Started" << std::endl;
}