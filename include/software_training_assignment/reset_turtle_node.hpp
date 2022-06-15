#pragma once
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <std_srvs/srv/empty.hpp>

namespace composition{
    class reset_turtle_node : public rclcpp::Node {
        public:
            reset_turtle_node(const rclcpp::NodeOptions &options);
        private:
            //make a service.... 
            //how do i do that lmao
            rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr client_;
            void reset();
            void reset_moving_turtle();
    };
}