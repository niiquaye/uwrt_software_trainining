#include "circular_turtles_node.hpp"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"



circular_turtle::circular_turtle(const rclcpp::NodeOptions &options) {
    //if this gets called then 

    //create a service
    timer_cb = create_wall_timer(2s, std::bind(rotate, this)); //turns the function into a function pointer
}

void rotate() {

    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;

    //moving a turtle needs a pub/sub relationship
    //need to create a publisher that publishes the velocity message to the turtle sim subscriber
    publisher = create_publisher<geometry_msgs::msg::Twist>("topic", 10);
    publisher -> publish(vel_msg); //should make the turtle run in a circle lmao
    
}