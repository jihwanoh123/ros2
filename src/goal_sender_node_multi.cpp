#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vector>

using namespace std::chrono_literals;

class MultiGoalSender : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    MultiGoalSender() : Node("multi_goal_sender"), current_goal_index_(0) {
        client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // Define multiple goals (x, y, orientation_w)
        goals_ = {
            {1.5, 1.5, 1.0},   // Goal 1
            {-1.0, 2.0, 0.707}, // Goal 2 (45° rotation)
            {0.0, -1.5, 0.0}    // Goal 3 (180° rotation)
        };

        timer_ = this->create_wall_timer(3s, std::bind(&MultiGoalSender::send_next_goal, this));
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    struct Goal {
        double x, y, orientation_w;
    };
    
    std::vector<Goal> goals_;
    size_t current_goal_index_;

    void send_next_goal() {
        if (current_goal_index_ >= goals_.size()) {
            RCLCPP_INFO(this->get_logger(), "All goals completed!");
            timer_->cancel();
            return;
        }

        if (!client_->wait_for_action_server(3s)) {
            RCLCPP_WARN(this->get_logger(), "Action server not ready");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->get_clock()->now();

        // Set current goal coordinates
        const auto& current_goal = goals_[current_goal_index_];
        goal_msg.pose.pose.position.x = current_goal.x;
        goal_msg.pose.pose.position.y = current_goal.y;
        goal_msg.pose.pose.orientation.w = current_goal.orientation_w;

        // Dynamic logging with goal number and coordinates
        RCLCPP_INFO(this->get_logger(), "Sending goal %zu/%zu to (%.2f, %.2f) with orientation %.3f", 
                   current_goal_index_ + 1, goals_.size(),
                   goal_msg.pose.pose.position.x, 
                   goal_msg.pose.pose.position.y,
                   goal_msg.pose.pose.orientation.w);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](const GoalHandleNav::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Goal %zu succeeded! Moving to next goal...", 
                           current_goal_index_ + 1);
                current_goal_index_++;
                // Schedule next goal after 2 seconds
                timer_ = this->create_wall_timer(2s, std::bind(&MultiGoalSender::send_next_goal, this));
            } else {
                RCLCPP_WARN(this->get_logger(), "Goal %zu failed or canceled. Skipping to next goal...", 
                           current_goal_index_ + 1);
                current_goal_index_++;
                timer_ = this->create_wall_timer(2s, std::bind(&MultiGoalSender::send_next_goal, this));
            }
        };

        client_->async_send_goal(goal_msg, send_goal_options);
        timer_->cancel(); // Cancel current timer, result callback will create new one
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiGoalSender>());
    rclcpp::shutdown();
    return 0;
} 