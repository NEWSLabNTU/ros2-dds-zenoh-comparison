#include <cstdio>
#include <thread>
#include "counter.hpp"


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto counter = std::make_shared<counter::ZenohCounter>();
    std::thread t1(counter::measure, counter);
    executor.add_node(counter);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
