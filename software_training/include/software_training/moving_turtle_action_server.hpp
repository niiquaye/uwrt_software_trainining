#ifndef MOVING_TURTLE_ACTION_SERVER_HPP_
#define MOVING_TURTLE_ACTION_SERVER_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>                 // ros2 time header
#include <rclcpp_action/rclcpp_action.hpp> // ros2 action header
#include <software_training/action/software.hpp>
#include <software_training/visibility.h>

#include <geometry_msgs/msg/twist.hpp> // cmd_vel publisher message
#include <turtlesim/msg/pose.hpp> // header for message to get moving turt position

namespace composition{

class moving_turtle_action_server : public rclcpp::Node {

public:
  SOFTWARE_TRAINING_PUBLIC
  explicit moving_turtle_action_server(const rclcpp::NodeOptions &options);

  // nice to have to prevent lengthy repitive code
  using GoalHandleActionServer =
      rclcpp_action::ServerGoalHandle<software_training::action::Software>;

private:
  // action server
  rclcpp_action::Server<software_training::action::Software>::SharedPtr
      action_server;

  // publisher to publish moving turtle commands
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

  // subscriber to get moving turt posiiton
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber;

  // goal callback function
  SOFTWARE_TRAINING_LOCAL
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const software_training::action::Software::Goal> goal);

  // cancel response callback function
  SOFTWARE_TRAINING_LOCAL
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleActionServer> goal_handle);

  // handle accepetd callback function
  SOFTWARE_TRAINING_LOCAL
  void
  handle_accepted(const std::shared_ptr<GoalHandleActionServer> goal_handle);

  // executioner callback function
  void execute(const std::shared_ptr<GoalHandleActionServer> goal_handle);

  // for subscriber
  static float x;
  static float y;
  static float theta;
  static float linear_velocity;
  static float angular_velocity;

  static constexpr unsigned int QUEUE{10};
};

}

#endif // MOVING_TURTLE_ACTION_SERVER_HPP_
