#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream> 
#include <chrono>
#include <limits>
#include <cmath>
#include <algorithm>
#include <atomic>


char getch_nonblocking() {
    char buf = 0;
    termios old{};
    if (tcgetattr(0, &old) < 0) return 0;
    termios new_t = old;
    new_t.c_lflag &= ~ICANON;
    new_t.c_lflag &= ~ECHO;
    new_t.c_cc[VMIN] = 0;  // Non-blocking
    new_t.c_cc[VTIME] = 0; // No timeout
    if (tcsetattr(0, TCSANOW, &new_t) < 0) return 0;
    if (read(0, &buf, 1) < 0) buf = 0;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) return 0;
    return buf;
}

class TeleopBot : public rclcpp::Node {
public:
    TeleopBot() : Node("teleop_bot") {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(), std::bind(&TeleopBot::scan_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
            std::bind(&TeleopBot::publish_cmd, this));
        startup_time = this->now();
        RCLCPP_INFO(this->get_logger(), "Smart Teleop Ready! Controls: w(forward), a(left), d(right), s(stop), q(quit)");
        RCLCPP_INFO(this->get_logger(), "Safety: Front detection 0.3m (±5°), Side detection 0.3m (±15°)");
    }

private:
    geometry_msgs::msg::Twist cmd_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

    bool safe_to_move = true;
    bool safe_to_move_left = true;
    bool safe_to_move_right = true;
    float front_distance = 0.0;
    bool received_scan = false;
    rclcpp::Time startup_time;
    
    // For faster response - track last command
    geometry_msgs::msg::Twist last_cmd_;
    bool has_pending_forward = false;
    std::atomic<bool> emergency_stop{false};

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Only monitor and update safety status - don't automatically control the robot
        int total_ranges = msg->ranges.size();
        
        // STEP 1: Always check directly in front first (essential for front collision)
        // In ROS2 laser scans: index 0 = front (0°), index size/2 = back (180°)
        const int FRONT_WINDOW = 10;  // ±10 points (~5°) directly ahead  
        float front_min_distance = std::numeric_limits<float>::max();
        bool front_obstacle = false;
        const float FRONT_SAFETY_DISTANCE = 0.3;  // Fixed safety for front detection
        
        // Check front sector: index 0 ± FRONT_WINDOW (wrapping around if needed)
        for (int offset = -FRONT_WINDOW; offset <= FRONT_WINDOW; ++offset) {
            int i = (offset + total_ranges) % total_ranges;  // Handle wrap-around
            if (std::isfinite(msg->ranges[i]) && msg->ranges[i] > 0.15 && msg->ranges[i] < 5.0) {
                front_min_distance = std::min(front_min_distance, msg->ranges[i]);
                if (msg->ranges[i] < FRONT_SAFETY_DISTANCE) {
                    front_obstacle = true;
                }
            }
        }
        
        // STEP 2: Wide-angle scan for side obstacles and overall situational awareness
        const int WIDE_WINDOW = 30;  // ±30 points (~15°) for side detection
        float wide_min_distance = std::numeric_limits<float>::max();
        bool side_obstacle = false;
        int obstacle_count = 0;
        const float SIDE_SAFETY_DISTANCE = 0.3;  // Can be closer to sides
        
        // Check wide front sector: index 0 ± WIDE_WINDOW (wrapping around if needed)
        for (int offset = -WIDE_WINDOW; offset <= WIDE_WINDOW; ++offset) {
            int i = (offset + total_ranges) % total_ranges;  // Handle wrap-around
            if (std::isfinite(msg->ranges[i]) && msg->ranges[i] > 0.15 && msg->ranges[i] < 5.0) {
                wide_min_distance = std::min(wide_min_distance, msg->ranges[i]);
                if (msg->ranges[i] < SIDE_SAFETY_DISTANCE) {
                    side_obstacle = true;
                    obstacle_count++;
                }
            }
        }
        
        // STEP 3: Combine front and side detection results
        bool obstacle_detected = front_obstacle || side_obstacle;
        float closest_obstacle = std::min(front_min_distance, wide_min_distance);
        
        // Use the most restrictive safety distance for reporting
        float effective_safety_distance = front_obstacle ? FRONT_SAFETY_DISTANCE : SIDE_SAFETY_DISTANCE;
        
        // Grace period after startup (2 seconds to settle)
        bool in_startup_grace = (this->now() - startup_time).seconds() < 2.0;
        
        // Update safety status
        safe_to_move = !obstacle_detected || in_startup_grace;
        front_distance = closest_obstacle < std::numeric_limits<float>::max() ? closest_obstacle : front_min_distance;
        received_scan = true;

        // IMMEDIATE emergency stop if moving forward into danger
        if (obstacle_detected && !in_startup_grace && has_pending_forward) {
            geometry_msgs::msg::Twist emergency_stop_cmd;
            emergency_stop_cmd.linear.x = 0.0;
            emergency_stop_cmd.angular.z = 0.0;
            publisher_->publish(emergency_stop_cmd);
            emergency_stop = true;
            RCLCPP_WARN(this->get_logger(), "⚡ EMERGENCY STOP! Immediate halt for obstacle at %.2f m", front_distance);
        } else if (!obstacle_detected) {
            emergency_stop = false;  // Reset emergency state when path is clear
        }

        // Update visualization markers
        publish_marker(obstacle_detected, front_distance, 0);
        
        // Enhanced logging with dual detection system info and angle verification
        std::string detection_type = front_obstacle ? "FRONT" : (side_obstacle ? "SIDE" : "CLEAR");
        float angle_increment_deg = msg->angle_increment * 180.0 / M_PI;
        float front_angle_span = FRONT_WINDOW * angle_increment_deg;
        float wide_angle_span = WIDE_WINDOW * angle_increment_deg;
        
        if (obstacle_detected) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "🚫 STOPPING! %s obstacle: %.2f m < %.2f m safety, Front(±%.1f°): %.2f m, Wide(±%.1f°): %d danger pts", 
                detection_type.c_str(), front_distance, effective_safety_distance, front_angle_span, front_min_distance, wide_angle_span, obstacle_count);
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "✅ Safe: Front(±%.1f°) %.2f m > %.2f m, Wide(±%.1f°) %.2f m > %.2f m", 
                front_angle_span, front_min_distance, FRONT_SAFETY_DISTANCE, wide_angle_span, wide_min_distance, SIDE_SAFETY_DISTANCE);
        }
    }

    void publish_cmd() {
        char c = getch_nonblocking();
        bool new_input = false;
        
        // Update command only if new input received
        if (c != 0) {
            new_input = true;
            switch (c) {
                case 'w': 
                    last_cmd_.linear.x = 0.3; 
                    last_cmd_.angular.z = 0.0;
                    has_pending_forward = true;
                    RCLCPP_INFO(this->get_logger(), "→ FORWARD command");
                    break;
                case 'a': 
                    last_cmd_.linear.x = 0.0;
                    last_cmd_.angular.z = 0.5; 
                    has_pending_forward = false;
                    RCLCPP_INFO(this->get_logger(), "↺ TURN LEFT command");
                    break;
                case 'd': 
                    last_cmd_.linear.x = 0.0;
                    last_cmd_.angular.z = -0.5; 
                    has_pending_forward = false;
                    RCLCPP_INFO(this->get_logger(), "↻ TURN RIGHT command");
                    break;
                case 's': 
                    last_cmd_.linear.x = 0.0;
                    last_cmd_.angular.z = 0.0; 
                    has_pending_forward = false;
                    RCLCPP_INFO(this->get_logger(), "🛑 STOP command");
                    break;
                case 'q': {
                    RCLCPP_INFO(this->get_logger(), "Quitting... Stopping robot.");
                    geometry_msgs::msg::Twist stop;
                    publisher_->publish(stop);
                    rclcpp::shutdown(); 
                    return;
                }
                default: 
                    new_input = false;
                    break;
            }
        }
        
        // Always execute current command (with safety checks)
        if (new_input || !emergency_stop) {
            // Safety check: only allow forward movement if path is clear
            if (last_cmd_.linear.x > 0.0) {  // Forward movement requested
                if (!received_scan) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "⚠️  No laser data - allowing movement");
                    publisher_->publish(last_cmd_);
                } else if (safe_to_move && !emergency_stop) {
                    publisher_->publish(last_cmd_);
                } else {
                    // Stop for safety - but don't spam logs
                    geometry_msgs::msg::Twist stop;
                    publisher_->publish(stop);
                    has_pending_forward = false;
                }
            } else {
                // Allow turning and stopping regardless of obstacles
                publisher_->publish(last_cmd_);
            }
        }
    }

    void publish_marker(bool obstacle_detected, float dist, int id) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->now();
        marker.ns = "obstacle_detection";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Position marker at detected distance
        marker.pose.position.x = std::min(dist, 2.0f);  // Cap at 2 meters for visualization
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.1;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;

        marker.color.a = 1.0;
        if (obstacle_detected) {
            // Red for obstacles
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        } else {
            // Green for clear path
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }

        marker.lifetime = rclcpp::Duration::from_seconds(0.5);
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
