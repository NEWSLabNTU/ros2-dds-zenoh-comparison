#include "pcd_generator.hpp"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pcd_generator::ZenohPcdGenerator>());
    rclcpp::shutdown();
    return 0;
}
