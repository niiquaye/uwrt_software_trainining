#ifndef RESET_MOVING_TURTLE_SERVICE_HPP_
#define RESET_MOVING_TURTLE_SERVICE_HPP_

#include <cstdlib>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <software_training/srv/software.hpp>
#include <software_training/visibility.h>
#include <turtlesim/srv/teleport_absolute.hpp>

namespace composition {

class reset_moving_turtle_service : public rclcpp::Node {

public:
  SOFTWARE_TRAINING_PUBLIC
  explicit reset_moving_turtle_service(const rclcpp::NodeOptions &options);

private:
  // create service that will reset turtle to starting point
  rclcpp::Service<software_training::srv::Software>::SharedPtr service;

  // create client
  rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr client;

  // server callback
  SOFTWARE_TRAINING_LOCAL
  void service_callback(
      const std::shared_ptr<software_training::srv::Software::Request> request,
      std::shared_ptr<software_training::srv::Software::Response> response);

  typedef struct reset {
    constexpr static float x = 5.44;
    constexpr static float y = 5.44;
    constexpr static float theta = 0;
  } reset_coordinates;
};

} // namespace composition

#endif // RESET_MOVING_TURTLE_SERVICE_HPP_
