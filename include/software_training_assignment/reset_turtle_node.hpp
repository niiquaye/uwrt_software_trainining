#pragma once
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
// #include <software_training_assignment/srv/Software.hpp>

namespace composition{
    class reset_turtle_node : public rclcpp::Node {
        public:
            reset_turtle_node(const rclcpp::NodeOptions &options);
        private:
            //make a service.... 
            //how do i do that lmao
            rclcpp::Service<software_training_assignment::srv::Software::Request>::SharedPtr service_;
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr client_;
            void reset();
            void reset_moving_turtle();
    };
}