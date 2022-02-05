#ifndef MOVING_TURTLE_ACTION_SERVER_HPP_
#define MOVING_TURTLE_ACTION_SERVER_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>                 // ros2 time header
#include <rclcpp_action/rclcpp_action.hpp> // ros2 action header
#include <rclcpp_components/register_node_macro.hpp>
#include <software_training_assignment/action/software.hpp>
#include <software_training_assignment/visibility.h>

#include <geometry_msgs/msg/twist.hpp> // cmd_vel publisher message
#include <turtlesim/msg/pose.hpp> // header for message to get moving turt position

namespace composition {

class MovingTurtleActionServer : public rclcpp::Node {

public:
    using Software = software_training_assignment::action::Software;
    using GoalHandleSoftware = rclcpp_action::ServerGoalHandle<Software>;

  SOFTWARE_TRAINING_PUBLIC
  explicit MovingTurtleActionServer(const rclcpp::NodeOptions &options);


private:
  // action server
  rclcpp_action::Server<Software>::SharedPtr
      action_server_;

  // publisher to publish moving turtle commands
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  // subscriber to get moving turt posiiton
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;

  // goal callback function
  SOFTWARE_TRAINING_LOCAL
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const Software::Goal> goal);

  // cancel response callback function
  SOFTWARE_TRAINING_LOCAL
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleSoftware> goal_handle);

  // handle accepetd callback function
  SOFTWARE_TRAINING_LOCAL
  void
  handle_accepted(const std::shared_ptr<GoalHandleSoftware> goal_handle);

  // executioner callback function
  SOFTWARE_TRAINING_LOCAL
  void execute(const std::shared_ptr<GoalHandleSoftware> goal_handle);

  SOFTWARE_TRAINING_LOCAL
  void subscriber_callback(const turtlesim::msg::Pose::SharedPtr msg);

  // for subscriber
  float x;
  float y;
  float theta;
  float linear_velocity;
  float angular_velocity;

  static constexpr unsigned int QUEUE{10};
};

}

#endif // MOVING_TURTLE_ACTION_SERVER_HPP_
