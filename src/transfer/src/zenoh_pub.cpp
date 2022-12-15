#include <cstdio>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <zenohc/zenoh.h>
#include "rclcpp/serialization.hpp"


using PC2 = sensor_msgs::msg::PointCloud2;

class Transfer : public rclcpp::Node {
public:
    Transfer() : Node("Transfer") {
        sub_topic = this->declare_parameter("sub_topic", "velodyne_points");
        pub_topic = this->declare_parameter("pub_topic", "my_pub_topic");
        pub = z_declare_publisher(z_loan(s), z_keyexpr(pub_topic.c_str()), NULL);
        sub = this->create_subscription<PC2>(
            sub_topic,
            10,
            std::bind(&Transfer::callback, this, std::placeholders::_1)
        );
    }

private:
    
    void callback(const PC2::UniquePtr msg) {
        auto pcd = *msg;
        rclcpp::SerializedMessage serialized_msg = rclcpp::SerializedMessage(0u);
        serializer.serialize_message(&pcd, &serialized_msg); 
        z_publisher_put(z_loan(pub), (const uint8_t *)serialized_msg.get_rcl_serialized_message().buffer, 
                        serialized_msg.get_rcl_serialized_message().buffer_length, NULL);
        RCLCPP_INFO(
            this->get_logger(),
            "Received message, %d x %d",
            msg->width,
            msg->height
        );
    }

    rclcpp::Subscription<PC2>::SharedPtr sub;
    std::string sub_topic, pub_topic;
    z_owned_config_t config = z_config_default();
    z_owned_session_t s = z_open(z_move(config));
    z_owned_publisher_t pub;
    rclcpp::Serialization<PC2> serializer; 
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Transfer>());
    rclcpp::shutdown();
    return 0;
}
