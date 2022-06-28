#pragma once

#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <cmath>
#include <turtlesim/msg/pose.hpp>

namespace composition {
class turtle_distance_node : public rclcpp::Node {
    public:
        turtle_distance_node(const rclcpp::NodeOptions &options);
    private:
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription;
        rclcpp::Publisher<software_training::msg::Software>::SharedPtr publisher;
        rclcpp::TimerBase::SharedPtr timer;
        float turtleDistance();

        float turtle1_x;
        float turtle1_y;
        float turtle2_x;
        float turtle2_y; 
};
}

