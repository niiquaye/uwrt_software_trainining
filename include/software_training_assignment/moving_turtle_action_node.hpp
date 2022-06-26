#include <cstdlib>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>

#include <turtlesim/msg/pose.hpp>
#include <software_training/action/software.hpp>

namespace composition{
class moving_turtle_action_node : public rclcpp::Node {
    public:

        moving_turtle_action_node();
        using GoalHandleActionServer =
            rclcpp_action::ServerGoalHandle<software_training::action::Software>;
    private:

        rclcpp_action::Server<software_training::action::Software>::SharedPtr action_server;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber;

        //copied some code from software training, since action server is hard to figure out
        SOFTWARE_TRAINING_LOCAL
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const software_training::action::Software::Goal> goal);

    
        SOFTWARE_TRAINING_LOCAL
        rclcpp_action::CancelResponse
        handle_cancel(const std::shared_ptr<GoalHandleActionServer> goal_handle);

        static constexpr unsigned int QUEUE{10};

};
}
