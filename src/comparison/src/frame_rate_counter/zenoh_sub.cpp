#include <cstdio>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/serialization.hpp>
#include <zenohc/zenoh.h>
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
        z_owned_config_t config = z_config_default();
        session = z_open(z_move(config));
        if (!z_check(session)) {
            printf("Unable to open session!\n");
            exit(-1);
        }

        // declare zenoh sub
        z_owned_closure_sample_t callback = z_closure(data_handler, NULL, this);
        z_subscriber_options_t opts = z_subscriber_options_default();
        opts.reliability = Z_RELIABILITY_RELIABLE;
        this->sub = z_declare_subscriber(
            z_loan(session),
            z_keyexpr(sub_topic.c_str()),
            z_move(callback),
            &opts
        );

        if (!z_check(sub)) {
            printf("Unable to declare Subscriber for key expression!\n");
            exit(-1);
        }
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
    z_owned_session_t session;
    z_owned_subscriber_t sub;

    static void data_handler(const z_sample_t* sample, void* counter_ptr) {
        Counter* counter = static_cast<Counter*>(counter_ptr);
        counter->recv++;

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