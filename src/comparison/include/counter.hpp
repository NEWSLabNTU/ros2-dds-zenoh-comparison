#ifndef COMPARISON_COUNTER
#define COMPARISON_COUNTER

#include <cstdio>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/serialization.hpp>


namespace counter {
    using PC2 = sensor_msgs::msg::PointCloud2;

    class BaseCounter : public rclcpp::Node {
    public:
        BaseCounter(std::string node_name) : rclcpp::Node(node_name) {
            this->recv.store(0);
            this->sub_topic = this->declare_parameter("sub_topic", "transfer_topic");
        }

        void print_stat(uint64_t elapsed) {
            uint64_t received = this->recv.exchange(0, std::memory_order::memory_order_relaxed);
            float interval = 1000000 / float(elapsed);
            if (received > 0) {
                RCLCPP_INFO(this->get_logger(), "FPS: %f", float(received)/interval);
            }
        }

        std::atomic<uint64_t> recv;
        std::string sub_topic;
    };

    void measure(std::shared_ptr<BaseCounter> counter) {
        while (true) {
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            std::this_thread::sleep_for(std::chrono::duration<int>(1));
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

            uint64_t elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
            counter->print_stat(elapsed);
        }
    }

    class RosCounter : public BaseCounter {
    public:
        RosCounter() : BaseCounter("ros_sub") {
            this->sub = this->create_subscription<PC2>(
                this->sub_topic,
                rclcpp::QoS(rclcpp::KeepAll()).reliable(),
                std::bind(&RosCounter::callback, this, std::placeholders::_1)
            );
        }

    private:


        rclcpp::Subscription<PC2>::SharedPtr sub;

        void callback(const PC2::UniquePtr msg) {
            (void)msg;
            this->recv++;
        }

    };
}

#endif
