#include <cstdio>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/serialization.hpp>
#include <zenohc/zenoh.h>
#include <chrono>


static const std::string NODE_NAME = "zenoh_pub";
static const rclcpp::Logger LOGGER = rclcpp::get_logger(NODE_NAME);

using PC2 = sensor_msgs::msg::PointCloud2;
using namespace std::chrono_literals;


std::shared_ptr<PC2> pc2_msg_;

void create_sample_msg() {
    pc2_msg_->header.frame_id = "velodyne";
    pc2_msg_->header.stamp.sec = 0;
    pc2_msg_->header.stamp.nanosec = 0;
    pc2_msg_->height = 1800;
    pc2_msg_->width = 32;
    
    std::vector<sensor_msgs::msg::PointField> pf(6);
    // sensor_msgs::msg::PointField tpf;
    // tpf.name = 'x'; tpf.offset = 0; tpf.datatype = 7; tpf.count = 1; pf[0] = tpf;
    // tpf.name = 'y'; tpf.offset = 4; tpf.datatype = 7; tpf.count = 1; pf[1] = tpf;
    // tpf.name = 'z'; tpf.offset = 8; tpf.datatype = 7; tpf.count = 1; pf[2] = tpf;
    // tpf.name = 'intensity'; tpf.offset = 12; tpf.datatype = 7; tpf.count = 1; pf[3] = tpf;
    // tpf.name = 'ring'; tpf.offset = 16; tpf.datatype = 4; tpf.count = 1; pf[4] = tpf;
    tpf.name = 'time'; tpf.offset = 18; tpf.datatype = 7; tpf.count = 1; pf[5] = tpf;
    pc2_msg_->fields = pf;
    pc2_msg_->is_bigendian = false;
    pc2_msg_->point_step = 22;
    pc2_msg_->row_step = 704;

    int size = 704 * 1800;
    std::vector<uint8_t> data_(size);
    pc2_msg_->data = data_;

    return;
}

class Transfer : public rclcpp::Node {
public:
    Transfer() : Node(NODE_NAME) {
        std::string pub_topic = this->declare_parameter("pub_topic", "transfer_topic");
        

        // open zenoh session
        z_owned_config_t config = z_config_default();
        session = z_open(z_move(config));
        if (!z_check(session)) {
            printf("Unable to open session!\n");
            exit(-1);
        }

        // declare zenoh pub
        z_publisher_options_t opts = z_publisher_options_default();
        opts.congestion_control = Z_CONGESTION_CONTROL_BLOCK;
        pub = z_declare_publisher(
            z_loan(session),
            z_keyexpr(pub_topic.c_str()),
            &opts
        );
        if (!z_check(pub)) {
            printf("Unable to declare Publisher for key expression!\n");
            exit(-1);
        }
    

        auto callback = [&]() {
            // serialize message
            auto message_payload_length = static_cast<size_t>(pc2_msg_->data.size());
            rclcpp::SerializedMessage serialized_msg;
            serialized_msg.reserve(message_header_length + message_payload_length);
            serializer.serialize_message(pc2_msg_.get(), &serialized_msg);

            z_publisher_put_options_t options = z_publisher_put_options_default();
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

        this->timer_ = this->create_wall_timer(5ms, callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<PC2>::SharedPtr sub;
    z_owned_session_t session;
    z_owned_publisher_t pub;
    rclcpp::Serialization<PC2> serializer;
    uint8_t message_header_length = 8u;
};

int main(int argc, char ** argv) {
    create_sample_msg();
    rclcpp::init(argc, argv);
    pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    rclcpp::spin(std::make_shared<Transfer>());
    rclcpp::shutdown();
    return 0;
}
