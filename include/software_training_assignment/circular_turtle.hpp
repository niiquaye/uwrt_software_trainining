//ppdirective that states the header only gets included once in the project

#pragma once 

#include <rclcpp.hpp>
#include <cstdlib>

class circular_turtle : public rclcpp:Node {
    public: 
        circular_turtle(const rclcpp::NodeOptions &options);
    private:
        rclcpp::Client<turtlesim::srv::Spawn> client;
        rclcpp::Client<turtlesim::srv::Spawn>::SharedRequest sharedReq;
        rclcpp::timer::WallTimer timer_cb;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;


}