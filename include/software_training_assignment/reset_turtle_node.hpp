#pragma once
#include <cstdlib>
#include <rclcpp>
#include <turtlesim/srv/TeleportAbsolute>

class reset_turtle_node : public rclcpp::Node {
    public:
        reset_turtle_node(const rclcpp::NodeOptions &options);
    private:
        //make a service.... 
        //how do i do that lmao
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Client<rclcpp::srv::TeleportAbsolute>::SharedPtr client;
        
} 