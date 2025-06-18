#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;

class GoalSender : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    GoalSender() : Node("goal_sender") {
        client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        timer_ = this->create_wall_timer(3s, std::bind(&GoalSender::send_goal, this));
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void send_goal() {
        if (!client_->wait_for_action_server(3s)) {
            RCLCPP_WARN(this->get_logger(), "Action server not ready");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->get_clock()->now();

        goal_msg.pose.pose.position.x = 1.5;
        goal_msg.pose.pose.position.y = 1.5;
        goal_msg.pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "Sending goal to (%.2f, %.2f)", 
                   goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [](const GoalHandleNav::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(rclcpp::get_logger("GoalSender"), "Goal succeeded!");
            } else {
                RCLCPP_WARN(rclcpp::get_logger("GoalSender"), "Goal failed or canceled.");
            }
        };

        client_->async_send_goal(goal_msg, send_goal_options);
        timer_->cancel();  // prevent sending again
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalSender>());
    rclcpp::shutdown();
    return 0;
}
