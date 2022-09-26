#include "rclcpp/rclcpp.hpp"

#include "sick_visionary_t_driver/node.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = std::make_shared<DriverNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}