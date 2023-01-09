#include <cstdio>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/serialization.hpp>
#include <yaml.h>
#include <chrono>


static const std::string NODE_NAME = "ros_pub";
static const rclcpp::Logger LOGGER = rclcpp::get_logger(NODE_NAME);

using PC2 = sensor_msgs::msg::PointCloud2;
using namespace std::chrono_literals;

std::shared_ptr<PC2> pc2_msg_;

void parse_yaml() {
    pc2_msg_->header.frame_id = "velodyne";
    pc2_msg_->header.stamp.sec = 0;
    pc2_msg_->header.stamp.nanosec = 0;
    pc2_msg_->height = 1800;
    pc2_msg_->width = 32;
    std::vector<sensor_msgs::msg::PointField> pf(6);
    sensor_msgs::msg::PointField tpf;
    tpf.name = 'x'; tpf.offset = 0; tpf.datatype = 7; tpf.count = 1;
    pf[0] = tpf;
    tpf.name = 'y'; tpf.offset = 4; tpf.datatype = 7; tpf.count = 1;
    pf[1] = tpf;
    tpf.name = 'z'; tpf.offset = 8; tpf.datatype = 7; tpf.count = 1;
    pf[2] = tpf;
    tpf.name = 'intensity'; tpf.offset = 12; tpf.datatype = 7; tpf.count = 1;
    pf[3] = tpf;
    tpf.name = 'ring'; tpf.offset = 16; tpf.datatype = 4; tpf.count = 1;
    pf[4] = tpf;
    tpf.name = 'time'; tpf.offset = 18; tpf.datatype = 7; tpf.count = 1;
    pf[5] = tpf;
    pc2_msg_->fields = pf;
    pc2_msg_->is_bigendian = false;
    pc2_msg_->point_step = 22;
    pc2_msg_->row_step = 704;
    int size = 704*1800;
    std::vector<uint8_t> data_(size);
    pc2_msg_->data = data_;

    return;
}

class Transfer : public rclcpp::Node {
public:
    Transfer() : Node(NODE_NAME) {
        std::string pub_topic = this->declare_parameter("pub_topic", "transfer_topic");
        auto qos = rclcpp::QoS(rclcpp::KeepAll()).reliable();
        this->pub = this->create_publisher<PC2>(pub_topic, qos);
        this->timer_ = this->create_wall_timer(5ms, std::bind(&Transfer::callback, this));
        // this->sub = this->create_subscription<PC2>(
        //     sub_topic,
        //     qos,
        //     std::bind(&Transfer::callback, this, std::placeholders::_1)
        // );
    }

private:
    // void callback(const PC2::UniquePtr msg) const {
    void callback() {

        // serialize message
        auto message_payload_length = static_cast<size_t>(pc2_msg_->data.size());
        rclcpp::SerializedMessage serialized_msg;
        serialized_msg.reserve(message_header_length + message_payload_length);
        serializer.serialize_message(pc2_msg_.get(), &serialized_msg);
        // serializer.serialize_message(&pc2_msg_, &serialized_msg);

        // publish the serialized_msg
        this->pub->publish(serialized_msg);
        // pub->publish(*pc2_msg_);

        // RCLCPP_INFO(
        //     LOGGER,
        //     "Received message, %d x %d",
        //     msg->width,
        //     msg->height
        // );
    }
    // rclcpp::Subscription<PC2>::SharedPtr sub;
    rclcpp::Publisher<PC2>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Serialization<PC2> serializer;
    uint8_t message_header_length = 8u;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    parse_yaml();
    rclcpp::spin(std::make_shared<Transfer>());
    // pc2_msg_ = (PC2*)malloc(sizeof(PC2));
    rclcpp::shutdown();
    return 0;
}
