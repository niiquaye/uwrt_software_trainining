#ifndef RESET_MOVING_TURTLE_SERVICE_HPP_
#define RESET_MOVING_TURTLE_SERVICE_HPP_

#include <cstdlib>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <software_training_assignment/srv/software.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>

namespace composition {

void reset(const std::shared_ptr<software_training_assignment::srv::Software::Request> request, std::shared_ptr<software_training_assignment::srv::Software::Response> response);

} // namespace composition

#endif // RESET_MOVING_TURTLE_SERVICE_HPP_
