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
        z_owned_config_t config = z_config_default();
        session = std::make_shared<zenoh::Session>(std::get<zenoh::Session>(zenoh::open(std::move(config))));
        // if (!z_check(session)) {
        //     printf("Unable to open session!\n");
        //     exit(-1);
        // }

        // // declare zenoh pub
        // z_publisher_options_t opts = z_publisher_options_default();
        // opts.congestion_control = Z_CONGESTION_CONTROL_BLOCK;
        // pub = z_declare_publisher(
        //     z_loan(session),
        //     z_keyexpr(pub_topic.c_str()),
        //     &opts
        // );
        // if (!z_check(pub)) {
        //     printf("Unable to declare Publisher for key expression!\n");
        //     exit(-1);
        // }

        // ros sub with zenoh pub as callback

        // auto subscriber = std::get<zenoh::Subscriber>(session.declare_subscriber(keyexpr, data_handler));
        auto qos = rclcpp::QoS(rclcpp::KeepAll()).reliable();

        zenoh::PublisherOptions options;
        options.set_congestion_control(zenoh::CongestionControl::Z_CONGESTION_CONTROL_BLOCK);
        auto publisher = std::get<zenoh::Publisher>(session->declare_publisher(pub_topic.c_str(), options));
        pub = std::make_shared<zenoh::Publisher>(std::move(publisher));
        sub = this->create_subscription<PC2>(sub_topic, qos, std::bind(&Transfer::callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<PC2>::SharedPtr sub;
    // zenoh::Session session;
    // z_owned_publisher_t pub;
    std::shared_ptr<zenoh::Session> session;
    std::shared_ptr<zenoh::Publisher> pub;
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

        auto inner_msg = serialized_msg.get_rcl_serialized_message();
        auto payload = zenoh::BytesView(inner_msg.buffer, inner_msg.buffer_length);
        // std::string value = "Test from zenoh";
        // z_publisher_put_options_t options = z_publisher_put_options_default();
        // options.encoding = z_encoding(Z_ENCODING_PREFIX_TEXT_PLAIN, NULL);
        pub->put(payload);
        // z_publisher_put(
        //     z_loan(pub),
        //     (const uint8_t *)serialized_msg.get_rcl_serialized_message().buffer,
        //     serialized_msg.size(),
        //     &options
        // );
    };
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Transfer>());
    rclcpp::shutdown();
    return 0;
}
