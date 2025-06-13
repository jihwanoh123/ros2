#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64_multi_array.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"  // using addition for now

class TopicToServiceBridge : public rclcpp::Node {
public:
    TopicToServiceBridge()
    : Node("topic_service_bridge") {
        sub_ = this->create_subscription<example_interfaces::msg::Int64MultiArray>(
            "add_input", 10,
            std::bind(&TopicToServiceBridge::callback, this, std::placeholders::_1)
        );

        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add");

        RCLCPP_INFO(this->get_logger(), "Bridge ready");
    }

private:
    rclcpp::Subscription<example_interfaces::msg::Int64MultiArray>::SharedPtr sub_;
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;

    void callback(const example_interfaces::msg::Int64MultiArray::SharedPtr msg) {
        if (msg->data.size() != 2) {
            RCLCPP_WARN(this->get_logger(), "Expected 2 numbers, got %ld", msg->data.size());
            return;
        }

        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Service not available");
            return;
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = msg->data[0];
        request->b = msg->data[1];

        RCLCPP_INFO(this->get_logger(), "Sending request: %lld + %lld", request->a, request->b);

        auto future = client_->async_send_request(
            request,
            [this](rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future) {
                try {
                    auto response = future.get();
                    RCLCPP_INFO(this->get_logger(), "Result: %lld", response->sum);
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                }
            }
        );
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TopicToServiceBridge>());
    rclcpp::shutdown();
    return 0;
}
