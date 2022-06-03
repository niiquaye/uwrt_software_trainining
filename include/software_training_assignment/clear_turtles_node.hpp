#pragma once

//get all turtle names and request to kill each one
//make a request to the turtle killing service

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/kill.hpp>
#include <cstdlib>



namespace composition {
class clear_turtles_node : public rclcpp::Node{
    public:
        clear_turtles_node(const rclcpp::NodeOptions &options);

    private:
        
        rclcpp::Client<turtlesim::srv::Kill>* client;
        rclcpp::Client<turtlesim::srv::Kill>::SharedRequest* sharedReq; //shared request
        std::vector<std::string> nodes = {"turtle1", "moving_turtle", "stationary_turtle"};
        rclcpp::timer::WallTimer timer_cb;
        void kill();

};
}
