#include <chrono>
#include <software_training/reset_moving_turtle_service.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace composition {

reset_moving_turtle_service::reset_moving_turtle_service(
    const rclcpp::NodeOptions &options)
    : Node("reset_moving_turtle_service", options) {

  // create client
  this->client = this->create_client<turtlesim::srv::TeleportAbsolute>(
      "/moving_turtle/teleport_absolute");

  // create service
  this->service = this->create_service<software_training::srv::Software>(
      "/reset_moving_turtle",
      std::bind(&reset_moving_turtle_service::service_callback, this, _1, _2));
}

void reset_moving_turtle_service::service_callback(
    const std::shared_ptr<software_training::srv::Software::Request> request,
    std::shared_ptr<software_training::srv::Software::Response> response) {

  (void)request; // request is not needed

  RCLCPP_INFO(this->get_logger(), "Starting ...");

  // make client call to reset turtle
  if (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "System Aborted");
      response->success = false;
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service is not available! Exit!");
    response->success = false;
    return;
  }

  auto client_request =
      std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();

  // fill request data
  client_request->x = reset_moving_turtle_service::reset_coordinates::x;
  client_request->y = reset_moving_turtle_service::reset_coordinates::y;
  client_request->theta = reset_moving_turtle_service::reset_coordinates::theta;

  // create response callback
  auto response_callback =
      [this](
          rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedFuture future)
      -> void {
    (void)future; // not needed
    RCLCPP_INFO(this->get_logger(), "Turtle Moved");
  };

  // send client request
  auto result = client->async_send_request(client_request, response_callback);

  RCLCPP_INFO(this->get_logger(), "Turtle Reseted");

  response->success = true;
}

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::reset_moving_turtle_service)
