#include <cmath>
#include <memory>
#include <software_training/moving_turtle_action_server.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace composition {

moving_turtle_action_server::moving_turtle_action_server(
    const rclcpp::NodeOptions &options)
    : Node("moving_turtle_action_server", options) {

  // create publisher
  this->publisher = this->create_publisher<geometry_msgs::msg::Twist>(
      "/moving_turtle/cmd_vel", rclcpp::QoS(QUEUE));

  // TODO: Enable topic statistics for subscriber

  auto subscriber_callback =
      [this](const turtlesim::msg::Pose::SharedPtr msg) -> void {
    this->moving_turtle_action_server::x = msg->x;
    this->moving_turtle_action_server::y = msg->y;
    this->moving_turtle_action_server::theta = msg->theta;
    this->moving_turtle_action_server::linear_velocity = msg->linear_velocity;
    this->moving_turtle_action_server::angular_velocity = msg->angular_velocity;
  };

  // create subscriber
  this->subscriber = this->create_subscription<turtlesim::msg::Pose>(
      "/moving_turtle/pose", QUEUE, subscriber_callback);

  // create action server
  this->action_server =
      rclcpp_action::create_server<software_training::action::Software>(
          this, "moving_turtle_action_server",
          std::bind(&moving_turtle_action_server::handle_goal, this, _1, _2),
          std::bind(&moving_turtle_action_server::handle_cancel, this, _1),
          std::bind(&moving_turtle_action_server::handle_accepted, this, _1)

      );
}

rclcpp_action::GoalResponse moving_turtle_action_server::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const software_training::action::Software::Goal> goal) {
  (void)uuid; // not needed - prevents compiler warnings/errors -Wall flag
  RCLCPP_INFO(this->get_logger(), "Goal Received");
  RCLCPP_INFO(this->get_logger(), "linear X:%f Y:%f Z:%f", goal->linear_pos.x,
              goal->linear_pos.y, goal->linear_pos.z);
  RCLCPP_INFO(this->get_logger(), "angular X:%f Y:%f Z:%f", goal->angular_pos.x,
              goal->angular_pos.y, goal->angular_pos.z);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse moving_turtle_action_server::handle_cancel(
    const std::shared_ptr<GoalHandleActionServer> goal_handle) {

  (void)goal_handle; // not needed

  RCLCPP_INFO(this->get_logger(), "Recieved Request to cancel goal!");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void moving_turtle_action_server::handle_accepted(
    const std::shared_ptr<GoalHandleActionServer> goal_handle) {

  // accept new goal and then 'execute' goal on a new thread to prevent blocking
  // action thus execute the goal on a different thread so no blocking action is
  // created detach thread so it can finish on its own

  std::thread{std::bind(&moving_turtle_action_server::execute, this, _1),
              goal_handle}
      .detach();
}

void moving_turtle_action_server::execute(
    const std::shared_ptr<GoalHandleActionServer> goal_handle) {

  rclcpp::Time start_time = this->now(); // get the current time

  RCLCPP_INFO(this->get_logger(), "Excuting Goal");

  rclcpp::Rate cycle_rate{1}; // set up frequency for goal execution

  // acquire the goal to be executed
  const auto goal = goal_handle->get_goal();

  // create feedback
  std::unique_ptr<software_training::action::Software::Feedback> feedback =
      std::make_unique<software_training::action::Software::Feedback>();

  // make result
  std::unique_ptr<software_training::action::Software::Result> result =
      std::make_unique<software_training::action::Software::Result>();

  // create reference to feedback for ease of use
  float &curr_x = feedback->x_pos;
  float &curr_y = feedback->y_pos;
  float &curr_theta = feedback->theta_pos;

  // keep track of linear feedback
  float lin_x{0};
  float lin_y{0};
  float lin_z{0};

  // keep track of angular feedback
  float ang_x{0};
  float ang_y{0};
  float ang_z{0};

  // heavy lifting
  while (rclcpp::ok() &&
         (lin_x < goal->linear_pos.x || lin_y < goal->linear_pos.y ||
          lin_z < goal->linear_pos.z || ang_x < goal->angular_pos.x ||
          ang_y < goal->angular_pos.y || ang_z < goal->angular_pos.z)) {

    // check if goal has been canceled
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal Canceled");

      // get the time it has taken thus far and update result
      rclcpp::Time curr_time = this->now();
      rclcpp::Duration time = curr_time - start_time;
      long int duration{time.nanoseconds()};
      result->duration = duration;

      goal_handle->canceled(std::move(result));
      return;
    }

    // publish to '/moving_turtle/cmd_vel' topic to move the turtle

    // create message
    auto message_cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();

    // fill message contents
    // if lin or ang distance points are already at target do not increment any
    // more just keep it the same hence the ternary operator
    message_cmd_vel->linear.x = (lin_x < goal->linear_pos.x) ? lin_x++ : lin_x;
    message_cmd_vel->linear.y = (lin_y < goal->linear_pos.y) ? lin_y++ : lin_y;
    message_cmd_vel->linear.z = (lin_z < goal->linear_pos.z) ? lin_z++ : lin_z;
    message_cmd_vel->angular.x =
        (ang_x < goal->angular_pos.x) ? ang_x++ : ang_x;
    message_cmd_vel->angular.y =
        (ang_y < goal->angular_pos.y) ? ang_y++ : ang_y;
    message_cmd_vel->angular.z =
        (ang_z < goal->angular_pos.z) ? ang_z++ : ang_z;

    // publish message
    this->publisher->publish(std::move(message_cmd_vel));

    // now compute feedback
    curr_x = this->moving_turtle_action_server::x - lin_x;
    curr_y = this->moving_turtle_action_server::y - lin_y;

    /********* MATH TO FIND THETA ************
     * theta = acos(z_axis_component/magnitude of vector)
     *
     *
     ********************************************/
    float theta{0};

    // scope this stuff cause we dont need it afterwards
    {

      float x1{lin_x}, x2{lin_y}, x3{lin_z};

      float magnitude{static_cast<float>(sqrt((x1 * x1) + (x2 * x2) + (x3 * x3)))};

      theta = acos(x3 / magnitude);
    }

    curr_theta = this->moving_turtle_action_server::theta - theta;

    // publish feedback
    goal_handle->publish_feedback(std::move(feedback));

    cycle_rate.sleep(); // control the rate at which the loop, loops through
  }

  // if goal is done
  if (rclcpp::ok()) {

    rclcpp::Time end = this->now();               // get end time
    rclcpp::Duration duration = end - start_time; // compute time taken
    long int res_time{duration.nanoseconds()};    // should be uint64_t

    // fill in result
    result->duration = res_time;

    // set the result
    goal_handle->succeed(
        std::move(result)); // move ownership so ptr is still usable
    RCLCPP_INFO(this->get_logger(), "Finish Executing Goal");
  }
}

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::moving_turtle_action_server)
