#include "transfer.hpp"


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<transfer::RosTransfer>());
    rclcpp::shutdown();
    return 0;
}
