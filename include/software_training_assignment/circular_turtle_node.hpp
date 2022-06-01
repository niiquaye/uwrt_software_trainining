//ppdirective that states the header only gets included once in the project

#pragma once 

#include <rclcpp.hpp>
#include <cstdlib>

class Vel_msg {
    class Velocity {
        private:
            _Float64 x,
            _Float64 y,
            _Float64 z
    }
    public:
       Velocity linear;
       Velocity angular; 
        
}

class circular_turtle : public rclcpp:Node {
    public: 
        circular_turtle(const rclcpp::NodeOptions &options);
    private:
        rclcpp::timer::WallTimer timer_cb;
        rclcpp::Publisher<geometry_msgs::msg::Twist>* publisher;
        //need to figure out how to publish a message to turtle1 outlining its angular velocity and all that
        Vel_msg Vel_msg;

}

