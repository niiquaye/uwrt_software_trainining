#ifndef TURTLE_PUBLISHER_HPP_
#define TURTLE_PUBLISHER_HPP_

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <software_training_assignment/msg/software.hpp>
#include <turtlesim/msg/pose.hpp>

namespace composition {

    class TurtlePublisher : public rclcpp::Node {
        public:
            explicit TurtlePublisher();

        private:
            void moving_callback(const turtlesim::msg::Pose::SharedPtr msg);
            void stationary_callback(const turtlesim::msg::Pose::SharedPtr msg);
            void publish();

            // position subscribers
            rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr stationary_sub;
            rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr moving_sub;

            // turtle publisher with custom message
            rclcpp::Publisher<software_training_assignment::msg::Software>::SharedPtr publisher_;

            // timer for publisher callback
            rclcpp::TimerBase::SharedPtr timer_;

            // callback groups - really just threads to run callbacks
            rclcpp::CallbackGroup::SharedPtr callbacks;

            float x_stationary;
            float y_stationary;

            float x_moving;
            float y_moving;
    };
} // namespace composition

#endif // TURTLE_PUBLISHER_HPP_
