#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

class AddTwoIntsServer : public rclcpp::Node {
public:
    AddTwoIntsServer()
    : Node("add_server") {
        // 1️⃣ CREATE THE SERVICE SERVER
        service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add",  // Service name
            std::bind(&AddTwoIntsServer::handle_request, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

private:
    // 2️⃣ DEFINE THE REQUEST HANDLER FUNCTION
    void handle_request(
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response
    ) {
        response->sum = request->a + request->b;  // Using multiplication instead of addition
        RCLCPP_INFO(this->get_logger(), "Received request: %lld + %lld = %lld", request->a, request->b, response->sum);
    }

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AddTwoIntsServer>());
    rclcpp::shutdown();
    return 0;
}
