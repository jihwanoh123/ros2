#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class VelocityPublisher : public rclcpp::Node {
public:
    VelocityPublisher() : Node("velocity_publisher") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
            std::bind(&VelocityPublisher::publish_velocity, this));
    }

private:
    void publish_velocity() {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.2;   // forward speed
        msg.angular.z = 0.0;  // no rotation
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing velocity: linear.x=%.2f", msg.linear.x);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityPublisher>());
    rclcpp::shutdown();
    return 0;
}
