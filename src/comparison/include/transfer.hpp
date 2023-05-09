#ifndef COMPARISON_TRANSFER
#define COMPARISON_TRANSFER

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/serialization.hpp>


namespace transfer {
    using PC2 = sensor_msgs::msg::PointCloud2;

    class BaseTransfer : public rclcpp::Node {
    public:
        BaseTransfer(std::string node_name) : rclcpp::Node(node_name) {
            this->sub_topic = this->declare_parameter("sub_topic", "velodyne_points");
            this->pub_topic = this->declare_parameter("pub_topic", "transfer_topic");
            this->sub = this->create_subscription<PC2>(
                this->sub_topic,
                this->qos,
                std::bind(&BaseTransfer::callback, this, std::placeholders::_1)
            );
        }

        rclcpp::Subscription<PC2>::SharedPtr sub;
        rclcpp::Serialization<PC2> serializer;
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepAll()).best_effort();
        std::string sub_topic, pub_topic;
        uint8_t message_header_length = 8u;
        virtual void callback(const PC2::UniquePtr) const {};
    };


    class RosTransfer : public BaseTransfer {
    public:
        RosTransfer() : BaseTransfer("ros_pub") {
            this->pub = this->create_publisher<PC2>(this->pub_topic, this->qos);
        }

    private:
        rclcpp::Publisher<PC2>::SharedPtr pub;
        void callback(const PC2::UniquePtr msg) const {
            // serialize message
            auto message_payload_length = static_cast<size_t>(msg->data.size());
            rclcpp::SerializedMessage serialized_msg;
            serialized_msg.reserve(this->message_header_length + message_payload_length);
            this->serializer.serialize_message(msg.get(), &serialized_msg);

            // publish the serialized_msg
            this->pub->publish(serialized_msg);

            // // Automatically serialized
            // pub->publish(*msg);
        }

    };
}

#endif
