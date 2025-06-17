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
    new_t.c_cc[VMIN] = 0;
    new_t.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &new_t) < 0) return 0;
    if (read(0, &buf, 1) < 0) buf = 0;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) return 0;
    return buf;
}

class TeleopBot : public rclcpp::Node {
public:
    TeleopBot() : Node("teleop_bot"), autonomous_mode_(false) {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(), std::bind(&TeleopBot::scan_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&TeleopBot::publish_cmd, this));
        startup_time = this->now();
        last_turn_time_ = this->now();
        wall_following_start_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Smart Teleop Ready! Controls: w/a/d/x, m(toggle mode)");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time startup_time;

    geometry_msgs::msg::Twist cmd_;
    bool autonomous_mode_;
    
    // State memory to prevent oscillation
    int last_turn_direction_ = 0;  // -1=right, 0=none, 1=left
    rclcpp::Time last_turn_time_;
    
    // Wall following persistence
    enum WallFollowingMode { NONE, LEFT_WALL, RIGHT_WALL };
    WallFollowingMode preferred_wall_ = NONE;
    rclcpp::Time wall_following_start_time_;

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!autonomous_mode_) return;

        int total_ranges = msg->ranges.size();
        
        // CORRECT laser indexing (ROS2 convention: index 0 = front, COUNTER-clockwise rotation)
        int front_idx = 0;
        int left_idx = total_ranges / 4;        // 90° counter-clockwise = LEFT side
        int right_idx = 3 * total_ranges / 4;   // 270° counter-clockwise = RIGHT side  
        int diagonal_left_idx = total_ranges / 8;        // 45° counter-clockwise = 45° left-front
        
        // Get distances with safety checks
        float front_dist = std::isfinite(msg->ranges[front_idx]) ? msg->ranges[front_idx] : 999.0;
        float left_dist = std::isfinite(msg->ranges[left_idx]) ? msg->ranges[left_idx] : 999.0;
        float right_dist = std::isfinite(msg->ranges[right_idx]) ? msg->ranges[right_idx] : 999.0;
        float diag_left_dist = std::isfinite(msg->ranges[diagonal_left_idx]) ? msg->ranges[diagonal_left_idx] : 999.0;
        
        // Wall following parameters
        const float DESIRED_WALL_DISTANCE = 0.4;  // Target distance from wall
        const float FRONT_COLLISION_THRESHOLD = 0.5;  // Stop if obstacle too close ahead
        const float CRITICAL_DISTANCE = 0.3;  // Very close - need backup
        const float WALL_DETECTION_THRESHOLD = 1.0;   // Max distance to consider a wall
        
        // Wall following state machine
        bool front_obstacle = front_dist < FRONT_COLLISION_THRESHOLD;
        bool wall_on_left = left_dist < WALL_DETECTION_THRESHOLD;
        bool wall_on_right = right_dist < WALL_DETECTION_THRESHOLD;
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Front: %.2f, Left: %.2f, Right: %.2f | Wall_L: %s, Wall_R: %s", 
            front_dist, left_dist, right_dist, 
            wall_on_left ? "YES" : "NO", wall_on_right ? "YES" : "NO");

        if (front_obstacle) {
            // PRIORITY 1: Avoid front collision - smart direction selection
            cmd_.linear.x = 0.0;
            
            // AGGRESSIVE anti-oscillation: commit to turning until clear
            auto current_time = this->now();
            double time_since_last_turn = (current_time - last_turn_time_).seconds();
            
            // If we're still seeing front obstacles after recent turn, keep turning MORE aggressively
            bool stuck_turning = time_since_last_turn < 3.0 && last_turn_direction_ != 0;
            
            int turn_direction;
            if (stuck_turning) {
                // COMMITTED TURNING: Don't change direction, just turn harder
                turn_direction = last_turn_direction_;
                cmd_.angular.z = turn_direction * 2.0;  // Extra aggressive turning
                
                // If very close, also backup
                if (front_dist < CRITICAL_DISTANCE) {
                    cmd_.linear.x = -0.15;  // Faster backup
                }
                
                RCLCPP_WARN(this->get_logger(), "🌪️ COMMITTED TURN: %s at %.1f rad/s (%.1fs elapsed)", 
                           turn_direction > 0 ? "LEFT" : "RIGHT", cmd_.angular.z, time_since_last_turn);
            } else {
                // Fresh obstacle detection - choose direction based on space
                if (left_dist < right_dist) {
                    turn_direction = -1;  // Turn right (towards more space)
                    RCLCPP_INFO(this->get_logger(), "🚫 NEW obstacle! Choose RIGHT (L:%.2f < R:%.2f)", left_dist, right_dist);
                } else {
                    turn_direction = 1;   // Turn left (towards more space)
                    RCLCPP_INFO(this->get_logger(), "🚫 NEW obstacle! Choose LEFT (R:%.2f < L:%.2f)", right_dist, left_dist);
                }
                
                // Apply initial turn speed
                cmd_.angular.z = turn_direction * 1.5;
                
                // If very close, backup
                if (front_dist < CRITICAL_DISTANCE) {
                    cmd_.linear.x = -0.1;
                    RCLCPP_WARN(this->get_logger(), "⚠️ CRITICAL distance! Backing up (Front: %.2f)", front_dist);
                }
                
                // Update memory
                last_turn_direction_ = turn_direction;
                last_turn_time_ = current_time;
            }
        } else if (wall_on_left || wall_on_right) {
            // PRIORITY 2: Wall following with persistence
            auto current_time = this->now();
            
            // Determine which wall to follow based on preference and availability
            bool should_follow_left = false;
            bool should_follow_right = false;
            
            if (preferred_wall_ == NONE) {
                // No preference yet - choose based on availability, prefer left (counter-clockwise)
                if (wall_on_left) {
                    preferred_wall_ = LEFT_WALL;
                    wall_following_start_time_ = current_time;
                    should_follow_left = true;
                    RCLCPP_INFO(this->get_logger(), "🔄 Starting LEFT wall following (counter-clockwise)");
                } else if (wall_on_right) {
                    preferred_wall_ = RIGHT_WALL;
                    wall_following_start_time_ = current_time;
                    should_follow_right = true;
                    RCLCPP_INFO(this->get_logger(), "🔄 Starting RIGHT wall following (clockwise)");
                }
            } else {
                // Have a preference - stick with it unless the preferred wall disappears for too long
                double time_following = (current_time - wall_following_start_time_).seconds();
                
                if (preferred_wall_ == LEFT_WALL) {
                    if (wall_on_left || time_following < 2.0) {
                        // Preferred wall available OR recently lost - keep following left
                        should_follow_left = true;
                        if (wall_on_left) wall_following_start_time_ = current_time; // Reset timer if wall found
                    } else {
                        // Left wall lost for too long, switch to right if available
                        if (wall_on_right) {
                            preferred_wall_ = RIGHT_WALL;
                            wall_following_start_time_ = current_time;
                            should_follow_right = true;
                            RCLCPP_WARN(this->get_logger(), "🔄 SWITCHING: Left wall lost, now following RIGHT wall");
                        }
                    }
                } else if (preferred_wall_ == RIGHT_WALL) {
                    if (wall_on_right || time_following < 2.0) {
                        // Preferred wall available OR recently lost - keep following right
                        should_follow_right = true;
                        if (wall_on_right) wall_following_start_time_ = current_time; // Reset timer if wall found
                    } else {
                        // Right wall lost for too long, switch to left if available
                        if (wall_on_left) {
                            preferred_wall_ = LEFT_WALL;
                            wall_following_start_time_ = current_time;
                            should_follow_left = true;
                            RCLCPP_WARN(this->get_logger(), "🔄 SWITCHING: Right wall lost, now following LEFT wall");
                        }
                    }
                }
            }
            
            // Execute wall following based on decision
            cmd_.linear.x = 0.2;
            
            if (should_follow_left) {
                float distance_error = left_dist - DESIRED_WALL_DISTANCE;
                if (distance_error > 0.2) {
                    cmd_.angular.z = 0.2;
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "↺ Too far from LEFT wall (%.2f), gentle left", left_dist);
                } else if (distance_error < -0.15) {
                    cmd_.angular.z = -0.2;
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "↻ Too close to LEFT wall (%.2f), gentle right", left_dist);
                } else {
                    cmd_.angular.z = distance_error * -0.3;
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "→ Following LEFT wall smoothly (%.2f)", left_dist);
                }
            } else if (should_follow_right) {
                float distance_error = right_dist - DESIRED_WALL_DISTANCE;
                if (distance_error > 0.2) {
                    cmd_.angular.z = -0.2;
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "↻ Too far from RIGHT wall (%.2f), gentle right", right_dist);
                } else if (distance_error < -0.15) {
                    cmd_.angular.z = 0.2;
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "↺ Too close to RIGHT wall (%.2f), gentle left", right_dist);
                } else {
                    cmd_.angular.z = distance_error * 0.3;
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "→ Following RIGHT wall smoothly (%.2f)", right_dist);
                }
            }
        } else {
            // PRIORITY 4: No walls detected - search for walls by moving forward and slowly turning
            cmd_.linear.x = 0.15;
            cmd_.angular.z = 0.2;  // Gentle left turn to search for walls
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "🔍 No walls detected, searching...");
        }
    }

    void publish_cmd() {
        char c = getch_nonblocking();
        if (c == 'm') {
            autonomous_mode_ = !autonomous_mode_;
            RCLCPP_INFO(this->get_logger(), "Mode switched to: %s", autonomous_mode_ ? "AUTONOMOUS" : "MANUAL");
        }

        if (!autonomous_mode_) {
            switch (c) {
                case 'w': cmd_.linear.x = 0.2; cmd_.angular.z = 0.0; break;
                case 'a': cmd_.angular.z = 0.5; cmd_.linear.x = 0.0; break;
                case 'd': cmd_.angular.z = -0.5; cmd_.linear.x = 0.0; break;
                case 'x': cmd_.linear.x = 0.0; cmd_.angular.z = 0.0; break;
                default: break;
            }
        }

        publisher_->publish(cmd_);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopBot>());
    rclcpp::shutdown();
    return 0;
}
