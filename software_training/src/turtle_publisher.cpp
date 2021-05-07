#include <cmath>
#include <iostream>
#include <software_training/turtle_publisher.hpp>
#include <string>

using namespace std::chrono_literals;

namespace composition {

turtle_publisher::turtle_publisher(const rclcpp::NodeOptions &options)
    : Node("turtle_publisher", options) {

  // callback for stationary turtle position
  auto stationary_turt_callback =
      [this](const turtlesim::msg::Pose::SharedPtr msg) -> void {
    this->x_stationary_turt = msg->x;
    this->y_stationary_turt = msg->y;
  };

  // callback for moving turtle position
  auto moving_turt_callback =
      [this](const turtlesim::msg::Pose::SharedPtr msg) -> void {
    this->x_moving_turt = msg->x;
    this->y_moving_turt = msg->y;
  };

  // publisher callback
  auto publisher_callback = [this](void) -> void {
    // compute absolute difference in coordinates
    double position_x{abs(this->x_stationary_turt - this->x_moving_turt)};
    double position_y{abs(this->y_stationary_turt - this->y_moving_turt)};

    // create message to publish
    auto msg = std::make_unique<software_training::msg::Software>();
    msg->x_pos = position_x;
    msg->y_pos = position_y;
    // compute distance using trig
    msg->distance = sqrt((position_x * position_x) + (position_y * position_y));

    // publish message
    this->publisher->publish(
        std::move(msg)); // must move message because it will be outscope and be
                         // freed because it is a smart pointer we do not want
                         // that - need to keep memory.
  };

  // create callback group - really threads that allow things to be executed on
  // that thread
  callbacks =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // create callback thread - so that everything with this callback will be on
  // one thread - this will ensure mutual exclusion
  auto callback_option = rclcpp::SubscriptionOptions();
  callback_option.callback_group = callbacks;

  // set ros2 topic statics
  callback_option.topic_stats_options.state =
      rclcpp::TopicStatisticsState::Enable;

  // set custom name for this topic statistic
  std::string s_name("/statistics/");
  std::string node_name(this->get_name());

  // topic name will be '/statistics/turtle_publisher' -- naming conventions
  // makes it easier to figure which statistics is for which node/component
  std::string stat_name = s_name + node_name;
  callback_option.topic_stats_options.publish_topic = stat_name.c_str();

  // instantiate subscribers
  stationary_turt_sub = this->create_subscription<turtlesim::msg::Pose>(
      "/stationary_turtle/pose", QUEUE, stationary_turt_callback,
      callback_option);

  moving_turt_sub = this->create_subscription<turtlesim::msg::Pose>(
      "/moving_turtle/pose", QUEUE, moving_turt_callback, callback_option);

  // instantiate publisher
  publisher = this->create_publisher<software_training::msg::Software>(
      "/difference", QUEUE);

  // set timer for publisher
  timer = this->create_wall_timer(3s, publisher_callback, callbacks);
}

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::turtle_publisher)
