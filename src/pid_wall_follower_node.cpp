#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "../include/pid_controller.hpp"
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

class PIDWallFollower : public rclcpp::Node {
public:
    PIDWallFollower() : Node("pid_wall_follower"), 
                       autonomous_mode_(false),
                       // PID Controllers for different behaviors
                       wall_distance_pid_(2.0, 0.1, 0.5, -1.5, 1.5, 2.0),  // Wall distance control
                       obstacle_avoidance_pid_(3.0, 0.0, 0.8, -2.0, 2.0),   // Obstacle avoidance
                       linear_speed_pid_(1.5, 0.05, 0.3, 0.0, 0.4, 1.0)     // Forward speed control
    {
        // Publishers and subscribers
        laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(), 
            std::bind(&PIDWallFollower::laser_callback, this, std::placeholders::_1));
        
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&PIDWallFollower::odom_callback, this, std::placeholders::_1));

        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
        debug_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pid_debug", 10);

        // Control timer (20Hz for smooth control)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), 
            std::bind(&PIDWallFollower::control_loop, this));
        
        startup_time_ = this->now();
        last_control_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "🤖 PID Wall Follower Ready!");
        RCLCPP_INFO(this->get_logger(), "Controls: m(toggle mode), w/a/d/x(manual), p(tune PID)");
        RCLCPP_INFO(this->get_logger(), "PID Gains - Wall: P=%.1f I=%.2f D=%.1f", 2.0, 0.1, 0.5);
    }

private:
    // ROS2 components
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // PID Controllers
    PIDController wall_distance_pid_;     // Controls angular velocity to maintain wall distance
    PIDController obstacle_avoidance_pid_; // Controls angular velocity for obstacle avoidance
    PIDController linear_speed_pid_;      // Controls forward speed based on path clearance
    
    // Robot state
    bool autonomous_mode_;
    geometry_msgs::msg::Twist manual_cmd_;
    rclcpp::Time startup_time_;
    rclcpp::Time last_control_time_;
    
    // Sensor data
    struct LaserData {
        float front_distance = 999.0;
        float left_distance = 999.0;
        float right_distance = 999.0;
        float front_left_distance = 999.0;
        float front_right_distance = 999.0;
        bool data_valid = false;
        rclcpp::Time timestamp;
    } laser_data_;
    
    struct OdomData {
        double x = 0.0, y = 0.0, yaw = 0.0;
        double linear_vel = 0.0, angular_vel = 0.0;
        bool data_valid = false;
    } odom_data_;
    
    // Wall following state
    enum WallFollowingMode { NONE, LEFT_WALL, RIGHT_WALL, OBSTACLE_AVOIDANCE };
    WallFollowingMode current_mode_ = NONE;
    WallFollowingMode preferred_mode_ = LEFT_WALL;  // Prefer left wall (counter-clockwise)
    
    // Control parameters
    const double DESIRED_WALL_DISTANCE = 0.5;     // Target distance from wall (meters)
    const double WALL_DETECTION_THRESHOLD = 1.2;   // Max distance to detect wall
    const double OBSTACLE_THRESHOLD = 0.6;         // Distance to consider obstacle
    const double CRITICAL_DISTANCE = 0.35;         // Emergency stop distance
    const double TARGET_LINEAR_SPEED = 0.25;       // Desired forward speed
    const double MIN_LINEAR_SPEED = 0.05;          // Minimum forward speed
    
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        int total_ranges = msg->ranges.size();
        
        // Calculate sector averages for more stable readings
        auto get_sector_min = [&](int center_idx, int window) -> float {
            float min_dist = 999.0;
            for (int offset = -window; offset <= window; ++offset) {
                int idx = (center_idx + offset + total_ranges) % total_ranges;
                if (std::isfinite(msg->ranges[idx]) && msg->ranges[idx] > 0.1) {
                    min_dist = std::min(min_dist, msg->ranges[idx]);
                }
            }
            return min_dist;
        };
        
        // Key directions
        int front_idx = 0;
        int left_idx = total_ranges / 4;      // 90° left
        int right_idx = 3 * total_ranges / 4; // 90° right (270° CCW)
        int front_left_idx = total_ranges / 8; // 45° left
        int front_right_idx = 7 * total_ranges / 8; // 45° right (315° CCW)
        
        // Get stable distance readings using sector minimums
        laser_data_.front_distance = get_sector_min(front_idx, 8);        // ±4° front
        laser_data_.left_distance = get_sector_min(left_idx, 6);          // ±3° left
        laser_data_.right_distance = get_sector_min(right_idx, 6);        // ±3° right  
        laser_data_.front_left_distance = get_sector_min(front_left_idx, 5);   // ±2.5° front-left
        laser_data_.front_right_distance = get_sector_min(front_right_idx, 5); // ±2.5° front-right
        
        laser_data_.data_valid = true;
        laser_data_.timestamp = this->now();
        
        // Publish visualization markers
        publish_laser_markers();
    }
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_data_.x = msg->pose.pose.position.x;
        odom_data_.y = msg->pose.pose.position.y;
        
        // Convert quaternion to yaw
        auto q = msg->pose.pose.orientation;
        odom_data_.yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        
        odom_data_.linear_vel = msg->twist.twist.linear.x;
        odom_data_.angular_vel = msg->twist.twist.angular.z;
        odom_data_.data_valid = true;
    }
    
    void control_loop() {
        // Handle keyboard input
        char c = getch_nonblocking();
        if (c == 'm') {
            autonomous_mode_ = !autonomous_mode_;
            if (autonomous_mode_) {
                // Reset PID controllers when entering autonomous mode
                wall_distance_pid_.reset();
                obstacle_avoidance_pid_.reset();
                linear_speed_pid_.reset();
            }
            RCLCPP_INFO(this->get_logger(), "🔄 Mode: %s", 
                       autonomous_mode_ ? "AUTONOMOUS" : "MANUAL");
        } else if (c == 'p') {
            tune_pid_gains();
        }
        
        // Calculate time step
        auto current_time = this->now();
        double dt = (current_time - last_control_time_).seconds();
        last_control_time_ = current_time;
        
        // Ensure reasonable time step
        if (dt > 0.2 || dt <= 0.0) dt = 0.05;  // Default to 20Hz if time step is unreasonable
        
        geometry_msgs::msg::Twist cmd;
        
        if (autonomous_mode_) {
            cmd = compute_autonomous_control(dt);
        } else {
            cmd = compute_manual_control(c);
        }
        
        cmd_publisher_->publish(cmd);
        publish_debug_info(dt);
    }
    
    geometry_msgs::msg::Twist compute_autonomous_control(double dt) {
        geometry_msgs::msg::Twist cmd;
        
        if (!laser_data_.data_valid) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "⚠️ No laser data - stopping");
            return cmd;  // Return zero velocity
        }
        
        // Determine current behavior mode
        WallFollowingMode new_mode = determine_behavior_mode();
        if (new_mode != current_mode_) {
            RCLCPP_INFO(this->get_logger(), "🔄 Mode change: %s → %s", 
                       mode_to_string(current_mode_).c_str(), 
                       mode_to_string(new_mode).c_str());
            current_mode_ = new_mode;
            
            // Reset relevant PID controllers on mode change
            wall_distance_pid_.reset();
            obstacle_avoidance_pid_.reset();
        }
        
        // Compute control based on current mode
        switch (current_mode_) {
            case OBSTACLE_AVOIDANCE:
                cmd = compute_obstacle_avoidance_control(dt);
                break;
            case LEFT_WALL:
                cmd = compute_wall_following_control(dt, true);  // true = left wall
                break;
            case RIGHT_WALL:
                cmd = compute_wall_following_control(dt, false); // false = right wall
                break;
            default:
                cmd = compute_exploration_control(dt);
                break;
        }
        
        return cmd;
    }
    
    WallFollowingMode determine_behavior_mode() {
        // Priority 1: Obstacle avoidance if front is blocked
        if (laser_data_.front_distance < OBSTACLE_THRESHOLD) {
            return OBSTACLE_AVOIDANCE;
        }
        
        // Priority 2: Wall following if walls detected
        bool left_wall_detected = laser_data_.left_distance < WALL_DETECTION_THRESHOLD;
        bool right_wall_detected = laser_data_.right_distance < WALL_DETECTION_THRESHOLD;
        
        if (left_wall_detected && right_wall_detected) {
            // Both walls detected - choose based on preference and which is closer to desired distance
            double left_error = std::abs(laser_data_.left_distance - DESIRED_WALL_DISTANCE);
            double right_error = std::abs(laser_data_.right_distance - DESIRED_WALL_DISTANCE);
            
            if (preferred_mode_ == LEFT_WALL && left_error < right_error + 0.1) {
                return LEFT_WALL;
            } else if (preferred_mode_ == RIGHT_WALL && right_error < left_error + 0.1) {
                return RIGHT_WALL;
            } else {
                // Choose the wall closer to desired distance
                return (left_error < right_error) ? LEFT_WALL : RIGHT_WALL;
            }
        } else if (left_wall_detected) {
            preferred_mode_ = LEFT_WALL;
            return LEFT_WALL;
        } else if (right_wall_detected) {
            preferred_mode_ = RIGHT_WALL;
            return RIGHT_WALL;
        }
        
        // Priority 3: No walls detected - exploration mode
        return NONE;
    }
    
    geometry_msgs::msg::Twist compute_obstacle_avoidance_control(double dt) {
        geometry_msgs::msg::Twist cmd;
        
        // Emergency stop if too close
        if (laser_data_.front_distance < CRITICAL_DISTANCE) {
            cmd.linear.x = -0.1;  // Backup slowly
            cmd.angular.z = 0.0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                "🚨 CRITICAL! Backing up (%.2fm)", laser_data_.front_distance);
            return cmd;
        }
        
        // Use PID to determine turn direction and magnitude
        double setpoint = OBSTACLE_THRESHOLD + 0.2;  // Desired clearance
        double angular_output = obstacle_avoidance_pid_.computeWithComponents(
            setpoint, laser_data_.front_distance, dt);
        
        // Determine turn direction based on side clearances
        double left_clearance = std::min(laser_data_.left_distance, laser_data_.front_left_distance);
        double right_clearance = std::min(laser_data_.right_distance, laser_data_.front_right_distance);
        
        // Turn towards the side with more clearance
        int turn_direction = (left_clearance > right_clearance) ? 1 : -1;
        cmd.angular.z = turn_direction * std::abs(angular_output);
        
        // Reduce forward speed when turning
        cmd.linear.x = std::max(MIN_LINEAR_SPEED, TARGET_LINEAR_SPEED * 0.3);
        
        // Debug output
        auto components = obstacle_avoidance_pid_.getLastComponents();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "🚫 Obstacle Avoid: Front=%.2f, Turn=%s, PID(P=%.2f I=%.2f D=%.2f)=%.2f",
                            laser_data_.front_distance, 
                            turn_direction > 0 ? "LEFT" : "RIGHT",
                            components.proportional, components.integral, 
                            components.derivative, angular_output);
        
        return cmd;
    }
    
    geometry_msgs::msg::Twist compute_wall_following_control(double dt, bool follow_left_wall) {
        geometry_msgs::msg::Twist cmd;
        
        // Get wall distance and front clearance
        double wall_distance = follow_left_wall ? laser_data_.left_distance : laser_data_.right_distance;
        double front_diagonal = follow_left_wall ? laser_data_.front_left_distance : laser_data_.front_right_distance;
        
        // PID control for wall distance
        double angular_output = wall_distance_pid_.computeWithComponents(
            DESIRED_WALL_DISTANCE, wall_distance, dt);
        
        // Apply turn direction based on which wall we're following
        cmd.angular.z = follow_left_wall ? -angular_output : angular_output;
        
        // PID control for forward speed based on front clearance
        double speed_setpoint = TARGET_LINEAR_SPEED;
        double front_clearance = std::min(static_cast<double>(laser_data_.front_distance), front_diagonal);
        
        if (front_clearance < OBSTACLE_THRESHOLD) {
            // Reduce speed when approaching obstacles
            speed_setpoint = TARGET_LINEAR_SPEED * (front_clearance / OBSTACLE_THRESHOLD);
        }
        
        cmd.linear.x = linear_speed_pid_.compute(speed_setpoint, odom_data_.linear_vel, dt);
        cmd.linear.x = std::clamp(cmd.linear.x, MIN_LINEAR_SPEED, TARGET_LINEAR_SPEED);
        
        // Debug output
        auto wall_components = wall_distance_pid_.getLastComponents();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "🧱 %s Wall: Dist=%.2f→%.2f, PID(P=%.2f I=%.2f D=%.2f)=%.2f, Speed=%.2f",
                            follow_left_wall ? "LEFT" : "RIGHT",
                            wall_distance, DESIRED_WALL_DISTANCE,
                            wall_components.proportional, wall_components.integral,
                            wall_components.derivative, angular_output, cmd.linear.x);
        
        return cmd;
    }
    
    geometry_msgs::msg::Twist compute_exploration_control(double /* dt */) {
        geometry_msgs::msg::Twist cmd;
        
        // Simple exploration: move forward and turn slightly to search for walls
        cmd.linear.x = TARGET_LINEAR_SPEED * 0.7;
        cmd.angular.z = 0.3;  // Gentle left turn to search for walls
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "🔍 Exploring: No walls detected, searching...");
        
        return cmd;
    }
    
    geometry_msgs::msg::Twist compute_manual_control(char c) {
        // Manual keyboard control
        switch (c) {
            case 'w': 
                manual_cmd_.linear.x = TARGET_LINEAR_SPEED; 
                manual_cmd_.angular.z = 0.0; 
                break;
            case 'a': 
                manual_cmd_.angular.z = 1.0; 
                manual_cmd_.linear.x = 0.0; 
                break;
            case 'd': 
                manual_cmd_.angular.z = -1.0; 
                manual_cmd_.linear.x = 0.0; 
                break;
            case 'x': 
                manual_cmd_.linear.x = 0.0; 
                manual_cmd_.angular.z = 0.0; 
                break;
            default: 
                break;
        }
        
        return manual_cmd_;
    }
    
    void tune_pid_gains() {
        // Cycle through different PID gain sets for testing
        static int gain_set = 0;
        gain_set = (gain_set + 1) % 3;
        
        switch (gain_set) {
            case 0:  // Conservative
                wall_distance_pid_.setGains(1.5, 0.05, 0.3);
                RCLCPP_INFO(this->get_logger(), "🎛️ PID: Conservative (P=1.5, I=0.05, D=0.3)");
                break;
            case 1:  // Balanced
                wall_distance_pid_.setGains(2.0, 0.1, 0.5);
                RCLCPP_INFO(this->get_logger(), "🎛️ PID: Balanced (P=2.0, I=0.1, D=0.5)");
                break;
            case 2:  // Aggressive
                wall_distance_pid_.setGains(3.0, 0.2, 0.8);
                RCLCPP_INFO(this->get_logger(), "🎛️ PID: Aggressive (P=3.0, I=0.2, D=0.8)");
                break;
        }
        
        // Reset PID state after gain change
        wall_distance_pid_.reset();
    }
    
    void publish_laser_markers() {
        // Publish visualization markers for key laser readings
        auto publish_marker = [&](double distance, double angle, int id, const std::string& color) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp = this->now();
            marker.ns = "laser_readings";
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position.x = distance * cos(angle);
            marker.pose.position.y = distance * sin(angle);
            marker.pose.position.z = 0.1;
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
            marker.color.a = 1.0;
            
            if (color == "red") {
                marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
            } else if (color == "green") {
                marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
            } else if (color == "blue") {
                marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0;
            }
            
            marker.lifetime = rclcpp::Duration::from_seconds(0.2);
            marker_publisher_->publish(marker);
        };
        
        // Publish markers for key directions
        publish_marker(std::min(static_cast<double>(laser_data_.front_distance), 3.0), 0.0, 0, "red");           // Front
        publish_marker(std::min(static_cast<double>(laser_data_.left_distance), 3.0), M_PI/2, 1, "green");       // Left
        publish_marker(std::min(static_cast<double>(laser_data_.right_distance), 3.0), -M_PI/2, 2, "blue");      // Right
    }
    
    void publish_debug_info(double dt) {
        std_msgs::msg::Float64MultiArray debug_msg;
        debug_msg.data = {
            laser_data_.front_distance,
            laser_data_.left_distance, 
            laser_data_.right_distance,
            static_cast<double>(current_mode_),
            wall_distance_pid_.getError(),
            wall_distance_pid_.getIntegral(),
            dt
        };
        debug_publisher_->publish(debug_msg);
    }
    
    std::string mode_to_string(WallFollowingMode mode) {
        switch (mode) {
            case NONE: return "EXPLORATION";
            case LEFT_WALL: return "LEFT_WALL";
            case RIGHT_WALL: return "RIGHT_WALL";
            case OBSTACLE_AVOIDANCE: return "OBSTACLE_AVOID";
            default: return "UNKNOWN";
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDWallFollower>());
    rclcpp::shutdown();
    return 0;
} 