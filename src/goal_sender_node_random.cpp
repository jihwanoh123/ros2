#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <random>

using namespace std::chrono_literals;

class RandomGoalSender : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    RandomGoalSender() : Node("random_goal_sender"), goal_count_(0) {
        client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // Initialize random number generator
        rng_.seed(std::chrono::steady_clock::now().time_since_epoch().count());
        x_dist_ = std::uniform_real_distribution<double>(-2.0, 2.0);  // X range: -2.0 to 2.0
        y_dist_ = std::uniform_real_distribution<double>(-2.0, 2.0);  // Y range: -2.0 to 2.0

        timer_ = this->create_wall_timer(3s, std::bind(&RandomGoalSender::send_random_goal, this));
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::mt19937 rng_;
    std::uniform_real_distribution<double> x_dist_, y_dist_;
    int goal_count_;

    void send_random_goal() {
        if (!client_->wait_for_action_server(3s)) {
            RCLCPP_WARN(this->get_logger(), "Action server not ready");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->get_clock()->now();

        // Generate random coordinates
        goal_msg.pose.pose.position.x = x_dist_(rng_);
        goal_msg.pose.pose.position.y = y_dist_(rng_);
        goal_msg.pose.pose.orientation.w = 1.0;

        goal_count_++;

        // Dynamic logging with random coordinates
        RCLCPP_INFO(this->get_logger(), "Sending random goal #%d to (%.2f, %.2f)", 
                   goal_count_,
                   goal_msg.pose.pose.position.x, 
                   goal_msg.pose.pose.position.y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](const GoalHandleNav::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Random goal #%d succeeded! Generating next goal in 5 seconds...", 
                           goal_count_);
                timer_ = this->create_wall_timer(5s, std::bind(&RandomGoalSender::send_random_goal, this));
            } else {
                RCLCPP_WARN(this->get_logger(), "Random goal #%d failed. Trying new goal in 3 seconds...", 
                           goal_count_);
                timer_ = this->create_wall_timer(3s, std::bind(&RandomGoalSender::send_random_goal, this));
            }
        };

        client_->async_send_goal(goal_msg, send_goal_options);
        timer_->cancel();
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RandomGoalSender>());
    rclcpp::shutdown();
    return 0;
} 