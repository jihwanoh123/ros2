#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream> 
#include <chrono>


char getch() {
    char buf = 0;
    termios old{};
    if (tcgetattr(0, &old) < 0) perror("tcsetattr()");
    termios new_t = old;
    new_t.c_lflag &= ~ICANON;
    new_t.c_lflag &= ~ECHO;
    new_t.c_cc[VMIN] = 1;
    new_t.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &new_t) < 0) perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0) perror ("read()");
    if (tcsetattr(0, TCSADRAIN, &old) < 0) perror ("tcsetattr ~ICANON");
    return buf;
}

class TeleopBot : public rclcpp::Node {
public:
    TeleopBot() : Node("teleop_bot") {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&TeleopBot::scan_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
            std::bind(&TeleopBot::publish_cmd, this));
        RCLCPP_INFO(this->get_logger(), "Use keys: w (forward), a (left), d (right), s (stop), q (quit)");
    }

private:
    geometry_msgs::msg::Twist cmd_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

    bool safe_to_move = true;

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        int center = msg->ranges.size() / 2;
        float dist = msg->ranges[center];
        safe_to_move = (dist >= 0.3);

        float left_dist = msg->ranges[center - 10];
        float right_dist = msg->ranges[center + 10];
        safe_to_move_left = (left_dist >= 0.3);
        safe_to_move_right = (right_dist >= 0.3);

        publish_marker(safe_to_move, dist);
        publish_marker(safe_to_move_left, left_dist);
        publish_marker(safe_to_move_right, right_dist);

        if (!safe_to_move) {
            RCLCPP_WARN(this->get_logger(), "Obstacle ahead at (%.2f meters)! STOPPING...", dist);
            cmd_.linear.x = 0.0;
            cmd_.angular.z = 0.0;
            safe_to_move = false;
        } else {
            RCLCPP_INFO(this->get_logger(), "Path clear, moving forward");
            RCLCPP_INFO(this->get_logger(), "Front center reading: %f", dist);
            cmd_.linear.x = 0.2;
            cmd_.angular.z = 0.0;
        }
        publisher_->publish(cmd_);
    }

    void publish_cmd() {
        char c = getch();
        geometry_msgs::msg::Twist user_cmd;
        switch (c) {
            case 'w': user_cmd.linear.x = 0.3; break;
            case 'a': user_cmd.angular.z = 0.5; break;
            case 'd': user_cmd.angular.z = -0.5; break;
            case 's': user_cmd.angular.z = 0.0; break;
            case 'q': rclcpp::shutdown(); return;
            default: return;
        }
        cmd_ = user_cmd;
        if (safe_to_move || cmd_.angular.z != 0.0) {
            publisher_->publish(cmd_);
        } else {
            geometry_msgs::msg::Twist stop;
            publisher_->publish(stop);
            RCLCPP_WARN(this->get_logger(), "Obstacle ahead or turning! Stopping...");
        }
    }

    void publish_marker(bool show_obstacle, float dist = 0.0) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";  // Use robot base frame
        marker.header.stamp = this->now();
        marker.ns = "obstacle_marker";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = show_obstacle ? visualization_msgs::msg::Marker::ADD
                                    : visualization_msgs::msg::Marker::DELETE;

        marker.pose.position.x = dist;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.1;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = show_obstacle ? 1.0 : 0.0;

        marker.lifetime = rclcpp::Duration::from_seconds(1.0);

        marker_publisher_->publish(marker);
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopBot>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
