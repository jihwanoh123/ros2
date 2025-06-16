#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserListener : public rclcpp::Node {
public:
    LaserListener() : Node("laser_listener") {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&LaserListener::scan_callback, this, std::placeholders::_1));
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(),
                    "LaserScan received:\n"
                    "Angle range: [%.2f, %.2f] (%.2f rad)\n"
                    "Readings: %zu\n"
                    "Range min: %.2f, max: %.2f\n"
                    "First range: %.2f",
                    msg->angle_min, msg->angle_max,
                    msg->angle_max - msg->angle_min,
                    msg->ranges.size(),
                    msg->range_min, msg->range_max,
                    msg->ranges.front());
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserListener>());
    rclcpp::shutdown();
    return 0;
}
