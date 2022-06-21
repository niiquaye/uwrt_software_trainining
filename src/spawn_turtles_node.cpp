#include "../include/software_training_assignment/spawn_turtles_node.hpp"

    
spawn_turtles::spawn_turtles(const rclcpp::NodeOptions &options) {
    timer_cb = create_wall_timer(2s, std::bind(spawn, this));
}

void spawn_turtles::spawn() {
    //make stationary turtle
    shared_request = std::make_shared<rclcpp::Client<rclcpp::srv::Spawn>::SharedRequest> (5, 5, 0, "stationary_turtle");

    auto callbackFunction1 = [] -> {
        std::cout << "callback1" << std::endl;
        shared_request = std::make_shared<rclcpp::Client<rclcpp::srv::Spawn>::SharedRequest> (25, 25, 0, "moving_turtle");
        client -> async_send_request(shared_request, callbackFunction2)
    }
    auto callbackFunction2 = [] -> {
        std::cout << "callback2" << std::endl;
    }
    client -> async_send_request(shared_request, callbackFunction1);

    

}