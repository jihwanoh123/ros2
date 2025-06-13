#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <iostream>
#include <memory>

class InteractiveClient : public rclcpp::Node {
public:
    InteractiveClient() : Node("interactive_client") {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add");
        wait_for_service();
        run_loop();
    }

private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;

    void wait_for_service() {
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for service to become available...");
        }
        RCLCPP_INFO(this->get_logger(), "Service available. Ready for input.");
    }

    void run_loop() {
        while (rclcpp::ok()) {
            int a, b;
            std::cout << "Enter two integers (a b), or Ctrl+C to exit: ";
            std::cin >> a >> b;

            auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
            request->a = a;
            request->b = b;

            auto future = client_->async_send_request(request);

            // Block until response comes
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future)
                == rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Result: %lld", future.get()->sum);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Service call failed");
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InteractiveClient>());
    rclcpp::shutdown();
    return 0;
}
