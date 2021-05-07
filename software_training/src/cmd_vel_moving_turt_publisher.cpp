#include <software_training/cmd_vel_moving_turt_publisher.hpp>

using namespace std::chrono_literals;

namespace composition {

cmd_vel_moving_turt_publisher::cmd_vel_moving_turt_publisher(
    const rclcpp::NodeOptions &options)
    : Node("cmd_vel_moving_turt_publisher", options) {

  auto publisher_callback = [this](void) -> void {
    auto message = std::make_unique<geometry_msgs::msg::Twist>();
    message->linear.x = cmd_vel_moving_turt_publisher::coordinates::linear::x;
    message->linear.y = cmd_vel_moving_turt_publisher::coordinates::linear::y;
    message->linear.z = cmd_vel_moving_turt_publisher::coordinates::linear::z;

    message->angular.x = cmd_vel_moving_turt_publisher::coordinates::angular::x;
    message->angular.y = cmd_vel_moving_turt_publisher::coordinates::angular::y;
    message->angular.z = cmd_vel_moving_turt_publisher::coordinates::angular::z;

    publisher->publish(std::move(message));
  };

  // create publisher
  // create a Quality of Service object with a history/ depth of 10 calls
  this->publisher = this->create_publisher<geometry_msgs::msg::Twist>(
      "/moving_turtle/cmd_vel", rclcpp::QoS(QUEUE));

  this->timer = this->create_wall_timer(1ms, publisher_callback);
}

} // namespace composition
#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::cmd_vel_moving_turt_publisher)
