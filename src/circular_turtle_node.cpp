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




    auto publish_callback = [this](void) -> void{
        auto message = std::make_unique<geometry_msgs::msg::Twist>();
        message->linear.x = circular_turtle::coordinates::linear::x;
        message->linear.y = circular_turtle::coordinates::linear::y;
        message->linear.z = circular_turtle::coordinates::linear::z;

        message->angular.x = circular_turtle::coordinates::angular::x;
        message->angular.y = circular_turtle::coordinates::angular::y;
        message->angular.z = circular_turtle::coordinates::angular::z;

    this->publisher->publish(std::move(message));
    };


    this -> publisher = this -> create_publisher<geometry_msgs::msg::Twist>("moving_turtle/cmd_vel", rclcpp::QoS(QUEUE)); //what do I put here lol
    timer_ = create_wall_timer(std::chrono::duration<int, std::chrono::seconds::period>(1), publish_callback);

    
  
    
}