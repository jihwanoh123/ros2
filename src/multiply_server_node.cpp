#include <rclcpp/rclcpp.hpp>
#include "matrix_publisher/srv/multiply_two_floats.hpp"

class MultiplyTwoFloatsServer : public rclcpp::Node {
public:
    MultiplyTwoFloatsServer() : Node("multiply_server") {
        service_ = this->create_service<matrix_publisher::srv::MultiplyTwoFloats>(
            "multiply",
            std::bind(&MultiplyTwoFloatsServer::handle_request, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

private:
    void handle_request(
        const std::shared_ptr<matrix_publisher::srv::MultiplyTwoFloats::Request> request,
        std::shared_ptr<matrix_publisher::srv::MultiplyTwoFloats::Response> response
    ) {
        response->product = request->a * request->b;
        RCLCPP_INFO(this->get_logger(), "Received request: %.2f * %.2f = %.2f", request->a, request->b, response->product);
    }

    rclcpp::Service<matrix_publisher::srv::MultiplyTwoFloats>::SharedPtr service_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiplyTwoFloatsServer>());
    rclcpp::shutdown();
    return 0;
} 