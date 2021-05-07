#ifndef CMD_VEL_MOVING_TURT_PUBLISHER_HPP_
#define CMD_VEL_MOVING_TURT_PUBLISHER_HPP_

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <software_training/visibility.h>

namespace composition {

class cmd_vel_moving_turt_publisher : public rclcpp::Node {

public:
  SOFTWARE_TRAINING_PUBLIC
  explicit cmd_vel_moving_turt_publisher(const rclcpp::NodeOptions &options);

private:
  // publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

  // callback timer
  rclcpp::TimerBase::SharedPtr timer;

  // set quality of service depth - AKA a backlog
  static constexpr unsigned int QUEUE{10};

  // namespace for values
  typedef struct point {

    typedef struct linear {
      static constexpr float x = 12;
      static constexpr float y = 12;
      static constexpr float z = 12;
    } linear;

    typedef struct angular {
      static constexpr float x = 1.41;
      static constexpr float y = 1.41;
      static constexpr float z = 1.41;

    } angular;

  } coordinates;
};

} // namespace composition

#endif //  CMD_VEL_MOVING_TURT_PUBLISHER_HPP_
