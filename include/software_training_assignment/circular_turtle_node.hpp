//ppdirective that states the header only gets included once in the project

#pragma once 

#include <rclcpp.hpp>
#include <cstdlib>

class circular_turtle : public rclcpp::Node {
    public: 
        circular_turtle(const rclcpp::NodeOptions &options);
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
        //need to figure out how to publish a message to turtle1 outlining its angular velocity and all that
        typedef struct point {

            typedef struct linear {
            static constexpr float x = 12;
            static constexpr float y = 12;
            static constexpr float z = 12;
            } linear;

            typedef struct angular {
            static constexpr float x = 1.41;
            static constexpr float y = 1.41;
            static constexpr float z = 1.41;

            } angular;

        } coordinates;
        void rotate();

};

