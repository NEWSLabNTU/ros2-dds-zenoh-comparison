#include <cstdio>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using PC2 = sensor_msgs::msg::PointCloud2;

class Transfer : public rclcpp::Node {
public:
    Transfer() : Node("Transfer") {
        sub_topic = this->declare_parameter("sub_topic", "velodyne_points");
        pub_topic = this->declare_parameter("pub_topic", "my_pub_topic");
        pub = this->create_publisher<PC2>(pub_topic, 10);
        sub = this->create_subscription<PC2>(
            sub_topic,
            10,
            std::bind(&Transfer::callback, this, std::placeholders::_1)
        );
    }

private:
    void callback(const PC2::UniquePtr msg) const {
        pub->publish(*msg);
        RCLCPP_INFO(
            this->get_logger(),
            "Received message, %d x %d",
            msg->width,
            msg->height
        );
    }
    rclcpp::Subscription<PC2>::SharedPtr sub;
    rclcpp::Publisher<PC2>::SharedPtr pub;
    std::string sub_topic, pub_topic;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Transfer>());
    rclcpp::shutdown();
    return 0;
}
