#include <cstdio>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/serialization.hpp>

static const std::string NODE_NAME = "ros_pub";
static const rclcpp::Logger LOGGER = rclcpp::get_logger(NODE_NAME);

using PC2 = sensor_msgs::msg::PointCloud2;

class Transfer : public rclcpp::Node {
public:
    Transfer() : Node(NODE_NAME) {
        std::string sub_topic = this->declare_parameter("sub_topic", "velodyne_points");
        std::string pub_topic = this->declare_parameter("pub_topic", "transfer_topic");
        pub = this->create_publisher<PC2>(pub_topic, 10);
        sub = this->create_subscription<PC2>(
            sub_topic,
            10,
            std::bind(&Transfer::callback, this, std::placeholders::_1)
        );
    }

private:
    void callback(const PC2::UniquePtr msg) const {
        // serialize message
        auto message_payload_length = static_cast<size_t>(msg->data.size());
        rclcpp::SerializedMessage serialized_msg;
        serialized_msg.reserve(message_header_length + message_payload_length);
        serializer.serialize_message(msg.get(), &serialized_msg);

        // publish the serialized_msg
        pub->publish(serialized_msg);
        // pub->publish(*msg);

        RCLCPP_INFO(
            LOGGER,
            "Received message, %d x %d",
            msg->width,
            msg->height
        );
    }
    rclcpp::Subscription<PC2>::SharedPtr sub;
    rclcpp::Publisher<PC2>::SharedPtr pub;
    rclcpp::Serialization<PC2> serializer;
    uint8_t message_header_length = 8u;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Transfer>());
    rclcpp::shutdown();
    return 0;
}
