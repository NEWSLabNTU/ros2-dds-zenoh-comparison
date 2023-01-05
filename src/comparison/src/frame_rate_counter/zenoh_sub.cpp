#include <cstdio>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/serialization.hpp>
#include "zenohcpp.h"
#include <chrono>
#include <thread>


static const std::string NODE_NAME = "zenoh_sub";
static const rclcpp::Logger LOGGER = rclcpp::get_logger(NODE_NAME);

using PC2 = sensor_msgs::msg::PointCloud2;
using SharedPub = rclcpp::Publisher<PC2>::SharedPtr;


class Counter : public rclcpp::Node {
public:
    Counter() : Node(NODE_NAME) {
        std::string sub_topic = this->declare_parameter("sub_topic", "transfer_topic");
        std::string pub_topic = this->declare_parameter("pub_topic", "frame_rate_counter");
        this->recv.store(0);

        // // declare ROS publisher
        // this->pub = this->create_publisher<PC2>(pub_topic, 10);

        // open zenoh session
        zenoh::Config config;
        this->session = std::make_unique<zenoh::Session>(std::get<zenoh::Session>(zenoh::open(std::move(config))));

        // declare zenoh sub
        zenoh::SubscriberOptions options;
        options.reliability = zenoh::Reliability::Z_RELIABILITY_RELIABLE;
        auto subscriber = std::get<zenoh::Subscriber>(this->session->declare_subscriber(
            sub_topic.c_str(),
            std::bind(&Counter::callback, this, std::placeholders::_1),
            options
        ));
        this->sub = std::make_unique<zenoh::Subscriber>(std::move(subscriber));
    }

    void print_stat(uint64_t elapsed) {
        uint64_t received = this->recv.exchange(0, std::memory_order::memory_order_relaxed);
        float interval = 1000000 / float(elapsed);
        if (received > 0) {
            RCLCPP_INFO(LOGGER, "FPS: %f", float(received)/interval);
        }
    }

private:
    // rclcpp::Publisher<PC2>::SharedPtr pub;
    std::atomic<uint64_t> recv;
    std::unique_ptr<zenoh::Session> session;
    std::unique_ptr<zenoh::Subscriber> sub;

    void callback(const z_sample_t* sample) {
        this->recv++;

        // manually create a serialized_msg from the zenoh payload
        rmw_serialized_message_t serialized_msg = rcutils_get_zero_initialized_uint8_array();
        serialized_msg.buffer = (uint8_t*)sample->payload.start;
        serialized_msg.buffer_length = sample->payload.len;
        serialized_msg.buffer_capacity = sample->payload.len;
        auto type_support = rosidl_typesupport_cpp::get_message_type_support_handle<PC2>();

        // start deserialization
        PC2 msg;
        if(rmw_deserialize(&serialized_msg, type_support, &msg) != RCUTILS_RET_OK) {
            RCLCPP_ERROR(LOGGER, "Failed to deserialize message!\n");
            return;
        }

        // counter->pub->publish(msg);
        // RCLCPP_INFO(LOGGER, "%d x %d\n", msg.width, msg.height);
    };


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
