#include <cstdio>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/serialization.hpp>
#include <chrono>
#include <thread>


static const std::string NODE_NAME = "ros_sub";
static const rclcpp::Logger LOGGER = rclcpp::get_logger(NODE_NAME);

using PC2 = sensor_msgs::msg::PointCloud2;
using SharedPub = rclcpp::Publisher<PC2>::SharedPtr;


class Counter : public rclcpp::Node {
public:
    Counter() : Node(NODE_NAME) {
        std::string sub_topic = this->declare_parameter("sub_topic", "transfer_topic");
        this->recv.store(0);

        // declare ROS sub
        auto qos = rclcpp::QoS(rclcpp::KeepAll()).reliable();
        this->sub = this->create_subscription<PC2>(
            sub_topic,
            qos,
            std::bind(&Counter::callback, this, std::placeholders::_1)
        );
    }

    void print_stat(uint64_t elapsed) {
        uint64_t received = this->recv.exchange(0, std::memory_order::memory_order_relaxed);
        float interval = 1000000 / float(elapsed);
        if (received > 0) {
            RCLCPP_INFO(LOGGER, "FPS: %f", float(received)/interval);
        }
    }

private:
    rclcpp::Subscription<PC2>::SharedPtr sub;
    std::atomic<uint64_t> recv;

    void callback(const PC2::UniquePtr msg) {
        (void)msg;
        this->recv++;
    }

};

void measure(std::shared_ptr<Counter> counter) {
    while (true) {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        std::this_thread::sleep_for(std::chrono::duration<int>(1));
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        uint64_t elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
        counter->print_stat(elapsed);
    }
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto counter = std::make_shared<Counter>();
    std::thread t1(measure, counter);
    executor.add_node(counter);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
