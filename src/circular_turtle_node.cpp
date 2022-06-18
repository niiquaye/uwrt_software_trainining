#include "../include/software_training_assignment/circular_turtle_node.hpp"
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace composition;

circular_turtle::circular_turtle(const rclcpp::NodeOptions &options) : rclcpp::Node("circular_turtle_node"){
    //if this gets called then 

    //create a service
    timer_ = create_wall_timer(std::chrono::duration<int, std::chrono::seconds::period>(1), std::bind(&circular_turtle::rotate, this));
   
}

void circular_turtle::rotate() {

    //moving a turtle needs a pub/sub relationship
    //need to create a publisher that publishes the velocity message to the turtle sim subscriber
    publisher = create_publisher<geometry_msgs::msg::Twist>("topic", 10);
    publisher -> publish(coordinates); //should make the turtle run in a circle lmao
    
}