#include <cstdio>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/serialization.hpp>
#include "zenohcpp.h"

static const std::string NODE_NAME = "zenoh_pub";
static const rclcpp::Logger LOGGER = rclcpp::get_logger(NODE_NAME);

using PC2 = sensor_msgs::msg::PointCloud2;

class Transfer : public rclcpp::Node {
public:
    Transfer() : Node(NODE_NAME) {
        std::string sub_topic = this->declare_parameter("sub_topic", "velodyne_points");
        std::string pub_topic = this->declare_parameter("pub_topic", "transfer_topic");

        // open zenoh session
        zenoh::Config config;
        this->session = std::make_unique<zenoh::Session>(std::get<zenoh::Session>(zenoh::open(std::move(config))));

        // zenoh pub
        zenoh::PublisherOptions options;
        options.set_congestion_control(zenoh::CongestionControl::Z_CONGESTION_CONTROL_BLOCK);
        auto publisher = std::get<zenoh::Publisher>(this->session->declare_publisher(pub_topic.c_str(), options));
        this->pub = std::make_unique<zenoh::Publisher>(std::move(publisher));

        // ros sub with zenoh pub as callback
        auto qos = rclcpp::QoS(rclcpp::KeepAll()).reliable();
        this->sub = this->create_subscription<PC2>(sub_topic, qos, std::bind(&Transfer::callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<PC2>::SharedPtr sub;
    std::unique_ptr<zenoh::Session> session;
    std::unique_ptr<zenoh::Publisher> pub;
    rclcpp::Serialization<PC2> serializer;
    uint8_t message_header_length = 8u;

    void callback(const PC2::UniquePtr msg) {
        RCLCPP_INFO(
            LOGGER,
            "Received message, %d x %d",
            msg->width,
            msg->height
        );

        // serialize message
        auto message_payload_length = static_cast<size_t>(msg->data.size());
        rclcpp::SerializedMessage serialized_msg;
        serialized_msg.reserve(message_header_length + message_payload_length);
        serializer.serialize_message(msg.get(), &serialized_msg);

        // zenoh put message
        auto inner_msg = serialized_msg.get_rcl_serialized_message();
        auto payload = zenoh::BytesView(inner_msg.buffer, inner_msg.buffer_length);
        pub->put(payload);
    };
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Transfer>());
    rclcpp::shutdown();
    return 0;
}
