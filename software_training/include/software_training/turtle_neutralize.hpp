#ifndef TURTLE_SERVICE_REQUEST_NODE_HPP_
#define TURTLE_SERVICE_REQUEST_NODE_HPP_

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <software_training/visibility.h>
#include <turtlesim/srv/kill.hpp>

namespace composition {

class turtle_service_request_node : public rclcpp::Node {
public:
  SOFTWARE_TRAINING_PUBLIC
  explicit turtle_service_request_node(const rclcpp::NodeOptions &options);

private:
  rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client;
  rclcpp::TimerBase::SharedPtr timer;

  // all the turtles
  std::vector<std::string> turtle_names = {"turtle1", "moving_turtle",
                                           "stationary_turtle"};

  SOFTWARE_TRAINING_LOCAL
  void kill();
};

} // namespace composition
#endif // TURTLE_SERVICE_REQUEST_NODE_HPP_
