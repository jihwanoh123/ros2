#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64_multi_array.hpp"
#include "matrix_publisher/srv/multiply_two_floats.hpp"
#include "std_msgs/msg/float64.hpp"

class TopicToServiceBridge : public rclcpp::Node {
public:
    TopicToServiceBridge()
    : Node("topic_service_bridge") {
        RCLCPP_INFO(this->get_logger(), "Creating subscription to /multiply_input");
        sub_ = this->create_subscription<example_interfaces::msg::Int64MultiArray>(
            "multiply_input", 10,
            std::bind(&TopicToServiceBridge::callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Creating publisher for /multiply_result");
        result_pub_ = this->create_publisher<std_msgs::msg::Float64>("multiply_result", 10);
        RCLCPP_INFO(this->get_logger(), "Creating client for /multiply service");
        client_ = this->create_client<matrix_publisher::srv::MultiplyTwoFloats>("multiply");
        RCLCPP_INFO(this->get_logger(), "Bridge ready");
    }

private:
    rclcpp::Subscription<example_interfaces::msg::Int64MultiArray>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr result_pub_;
    rclcpp::Client<matrix_publisher::srv::MultiplyTwoFloats>::SharedPtr client_;

    void callback(const example_interfaces::msg::Int64MultiArray::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received message with %ld elements", msg->data.size());
        if (msg->data.size() != 2) {
            RCLCPP_WARN(this->get_logger(), "Expected 2 numbers, got %ld", msg->data.size());
            return;
        }
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Service not available");
            return;
        }
        auto request = std::make_shared<matrix_publisher::srv::MultiplyTwoFloats::Request>();
        request->a = static_cast<double>(msg->data[0]);
        request->b = static_cast<double>(msg->data[1]);
        RCLCPP_INFO(this->get_logger(), "Sending request: %.2f * %.2f", request->a, request->b);
        auto future = client_->async_send_request(
            request,
            [this](rclcpp::Client<matrix_publisher::srv::MultiplyTwoFloats>::SharedFuture future) {
                try {
                    auto response = future.get();
                    RCLCPP_INFO(this->get_logger(), "Result: %.2f", response->product);
                    auto result_msg = std_msgs::msg::Float64();
                    result_msg.data = response->product;
                    result_pub_->publish(result_msg);
                    RCLCPP_INFO(this->get_logger(), "Published result: %.2f", response->product);
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
