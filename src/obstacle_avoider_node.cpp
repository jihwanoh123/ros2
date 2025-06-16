#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ObstacleAvoidance : public rclcpp::Node {
public:
    ObstacleAvoidance() : Node("obstacle_avoider") {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ObstacleAvoidance::scan_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // 1. focus on the front 30 degrees
        int center = msg->ranges.size() / 2;
        int window = 15;
        bool obstacle_too_close = false;

        for (int i = center - window; i < center + window; ++i) {
            float dist = msg->ranges[i];
            if (std::isfinite(dist) && dist < 0.3) {
                obstacle_too_close = true;
                break;
            }
        }

        geometry_msgs::msg::Twist cmd;
        if (obstacle_too_close) {
            RCLCPP_INFO(this->get_logger(), "Obstacle detected! Stopping...");
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.5;
        } else {
            RCLCPP_INFO(this->get_logger(), "Path clear, moving forward");
            RCLCPP_INFO(this->get_logger(), "Front center reading: %f", msg->ranges[center]);
            cmd.linear.x = 0.2;
            cmd.angular.z = 0.0;
        }
        publisher_->publish(cmd);
    }
};



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoidance>());
    rclcpp::shutdown();
    return 0;
}
