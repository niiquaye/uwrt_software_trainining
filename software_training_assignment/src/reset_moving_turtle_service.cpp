#include <software_training_assignment/reset_moving_turtle_service.hpp>

namespace composition {

using namespace std::chrono_literals;

void reset(const std::shared_ptr<software_training_assignment::srv::Software::Request> request,
          std::shared_ptr<software_training_assignment::srv::Software::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request to reset turtle");

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("teleport_turtle_client");
  rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr client =
    node->create_client<turtlesim::srv::TeleportAbsolute>("/moving_turtle/teleport_absolute");

  auto new_request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
  new_request->x = 25;
  new_request->y = 10;
  new_request->theta = 0;

  if (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      response->success = false;
      return; 
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "turtle teleporting service not available");
  }

  auto result = client->async_send_request(new_request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "teleported turtle!");
    response->success = true;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service teleport_absolute");
    response->success = false;
  }
}
} // namespace composition
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("reset_moving_turtle_server");

  rclcpp::Service<software_training_assignment::srv::Software>::SharedPtr service =
    node->create_service<software_training_assignment::srv::Software>("/reset_moving_turtle", &composition::reset);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to reset turtles.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
