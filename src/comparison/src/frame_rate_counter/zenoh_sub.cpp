#include <cstdio>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/serialization.hpp>
#include <zenohc/zenoh.h>

static const std::string NODE_NAME = "zenoh_sub";
static const rclcpp::Logger LOGGER = rclcpp::get_logger(NODE_NAME);

using PC2 = sensor_msgs::msg::PointCloud2;
using SharedPub = rclcpp::Publisher<PC2>::SharedPtr;


void data_handler(const z_sample_t* sample, void* pub) {
    SharedPub* pub_ptr = static_cast<SharedPub*>(pub);

    // manually create a serialized_msg from the zenoh payload
    rmw_serialized_message_t serialized_msg = rcutils_get_zero_initialized_uint8_array();
    serialized_msg.buffer = (uint8_t*)sample->payload.start;
    serialized_msg.buffer_length = sample->payload.len;
    serialized_msg.buffer_capacity = sample->payload.len;
    auto type_support = rosidl_typesupport_cpp::get_message_type_support_handle<PC2>();
    rclcpp::Serialization<PC2> serializer;

    // start deserialization
    PC2 msg;
    if(rmw_deserialize(&serialized_msg, type_support, &msg) != RCUTILS_RET_OK) {
        RCLCPP_ERROR(LOGGER, "Failed to deserialize message!\n");
        return;
    }

    (*pub_ptr)->publish(serialized_msg);
    RCLCPP_INFO(LOGGER, "%d x %d\n", msg.width, msg.height);
};

class Transfer : public rclcpp::Node {
public:
    Transfer() : Node(NODE_NAME) {
        std::string sub_topic = this->declare_parameter("sub_topic", "transfer_topic");
        std::string pub_topic = this->declare_parameter("pub_topic", "frame_rate_counter");

        // declare ROS publisher
        pub = this->create_publisher<PC2>(pub_topic, 10);

        // open zenoh session
        z_owned_config_t config = z_config_default();
        session = z_open(z_move(config));
        if (!z_check(session)) {
            printf("Unable to open session!\n");
            exit(-1);
        }

        // declare zenoh pub
        z_owned_closure_sample_t callback = z_closure(data_handler, NULL, &pub);
        sub = z_declare_subscriber(
            z_loan(session),
            z_keyexpr(sub_topic.c_str()),
            z_move(callback),
            NULL
        );

        if (!z_check(sub)) {
            printf("Unable to declare Subscriber for key expression!\n");
            exit(-1);
        }
    }

private:
    rclcpp::Publisher<PC2>::SharedPtr pub;
    z_owned_session_t session;
    z_owned_subscriber_t sub;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Transfer>());
    rclcpp::shutdown();
    return 0;
}
