#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

using namespace std::chrono_literals;

class AdderClient : public rclcpp::Node {
public:
    AdderClient(int a, int b) : Node("adder_client"), a_(a), b_(b) {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add");

        timer_ = this->create_wall_timer(
            1s, std::bind(&AdderClient::send_request, this));
    }

private:
    int a_, b_;
    void send_request() {
        if (!client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Waiting for service...");
            return;
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a_;
        request->b = b_;

        auto future = client_->async_send_request(request);

        // Wait for result (blocking, for simplicity)
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Result: %lld", future.get()->sum);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive response.");
        }

        // Optional: shutdown after one call
        rclcpp::shutdown();
    }

    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    if (argc < 3) {
        std::cerr << "Usage: adder_client_node <a> <b>\n";
        return 1;
    }

    int a = std::stoi(argv[1]);
    int b = std::stoi(argv[2]);

    auto node = std::make_shared<AdderClient>(a, b);
    rclcpp::spin(node);
    return 0;
}
