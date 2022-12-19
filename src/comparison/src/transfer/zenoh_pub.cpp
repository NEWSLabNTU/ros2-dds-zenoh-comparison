#include <cstdio>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/serialization.hpp>
#include <zenohc/zenoh.h>

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
        session = z_open(z_move(config));
        if (!z_check(session)) {
            printf("Unable to open session!\n");
            exit(-1);
        }

        // declare zenoh pub
        pub = z_declare_publisher(z_loan(session), z_keyexpr(pub_topic.c_str()), NULL);
        if (!z_check(pub)) {
            printf("Unable to declare Publisher for key expression!\n");
            exit(-1);
        }

        auto callback = [&](const PC2::UniquePtr msg) {
            // serialize message
            auto message_payload_length = static_cast<size_t>(msg->data.size());
            rclcpp::SerializedMessage serialized_msg;
            serialized_msg.reserve(message_header_length + message_payload_length);
            serializer.serialize_message(msg.get(), &serialized_msg);

            // std::string value = "Test from zenoh";
            z_publisher_put_options_t options = z_publisher_put_options_default();
            // options.encoding = z_encoding(Z_ENCODING_PREFIX_TEXT_PLAIN, NULL);
            z_publisher_put(
                z_loan(pub),
                (const uint8_t *)serialized_msg.get_rcl_serialized_message().buffer,
                serialized_msg.size(),
                &options
            );
            // RCLCPP_INFO(
            //     LOGGER,
            //     "Received message, %d x %d",
            //     msg->width,
            //     msg->height
            // );
        };

        // ros sub with zenoh pub as callback
        auto qos = rclcpp::QoS(rclcpp::KeepAll()).reliable();
        sub = this->create_subscription<PC2>(sub_topic, qos, callback);
    }

private:
    rclcpp::Subscription<PC2>::SharedPtr sub;
    z_owned_session_t session;
    z_owned_publisher_t pub;
    rclcpp::Serialization<PC2> serializer;
    uint8_t message_header_length = 8u;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Transfer>());
    rclcpp::shutdown();
    return 0;
}
